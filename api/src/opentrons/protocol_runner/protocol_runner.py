"""Protocol run control and management."""
import asyncio
from typing import List, NamedTuple, Optional, Union

from abc import ABC, abstractmethod

import anyio

from opentrons.broker import Broker
from opentrons.equipment_broker import EquipmentBroker
from opentrons.hardware_control import HardwareControlAPI
from opentrons import protocol_reader
from opentrons.protocol_reader import (
    ProtocolSource,
    JsonProtocolConfig,
    PythonProtocolConfig,
)
from opentrons.protocol_engine import ProtocolEngine, StateSummary, Command

from .task_queue import TaskQueue
from .json_file_reader import JsonFileReader
from .json_translator import JsonTranslator
from .legacy_context_plugin import LegacyContextPlugin
from .legacy_wrappers import (
    LEGACY_PYTHON_API_VERSION_CUTOFF,
    LEGACY_JSON_SCHEMA_VERSION_CUTOFF,
    LegacyFileReader,
    LegacyContextCreator,
    LegacyExecutor,
    LegacyLoadInfo,
)


class ProtocolRunResult(NamedTuple):
    """Result data from a run, pulled from the ProtocolEngine."""

    commands: List[Command]
    state_summary: StateSummary


class AbstractRunner(ABC):
    """An interface to manage and control a protocol run.

    The AbstractRunner is primarily responsible for feeding a ProtocolEngine
    with commands and control signals. These commands and signals are
    generated by protocol files, hardware signals, or externally via
    the HTTP robot-server.

    A AbstractRunner controls a single run. Once the run is finished,
    you will need a new AbstractRunner to do another run.
    """

    @abstractmethod
    def was_started(self) -> bool:
        """Whether the runner has been started.

        This value is latched; once it is True, it will never become False.
        """

    @abstractmethod
    async def load(self, protocol_source: ProtocolSource) -> None:
        """Load a ProtocolSource into managed ProtocolEngine.

        Calling this method is only necessary if the runner will be used
        to control the run of a file-based protocol.
        """

    @abstractmethod
    def play(self) -> None:
        """Start or resume the run."""

    @abstractmethod
    def pause(self) -> None:
        """Pause the run."""

    @abstractmethod
    async def stop(self) -> None:
        """Stop (cancel) the run."""

    @abstractmethod
    async def run(
        self,
        protocol_source: Optional[ProtocolSource] = None,
    ) -> ProtocolRunResult:
        """Run a given protocol to completion."""


class PythonAndLegacyRunner(AbstractRunner):
    """Protocol runner implementation for Python protocols, and JSON protocols ≤v5."""

    def __init__(
        self,
        protocol_engine: ProtocolEngine,
        hardware_api: HardwareControlAPI,
        task_queue: Optional[TaskQueue] = None,
        legacy_file_reader: Optional[LegacyFileReader] = None,
        legacy_context_creator: Optional[LegacyContextCreator] = None,
        legacy_executor: Optional[LegacyExecutor] = None,
    ) -> None:
        """Initialize the PythonAndLegacyRunner with its dependencies."""
        self._protocol_engine = protocol_engine
        self._hardware_api = hardware_api
        self._legacy_file_reader = legacy_file_reader or LegacyFileReader()
        self._legacy_context_creator = legacy_context_creator or LegacyContextCreator(
            hardware_api=hardware_api,
            protocol_engine=protocol_engine,
        )
        self._legacy_executor = legacy_executor or LegacyExecutor()
        # TODO(mc, 2022-01-11): replace task queue with specific implementations
        # of runner interface
        self._task_queue = task_queue or TaskQueue(cleanup_func=protocol_engine.finish)

    def was_started(self) -> bool:
        return self._protocol_engine.state_view.commands.has_been_played()

    async def load(self, protocol_source: ProtocolSource) -> None:
        labware_definitions = await protocol_reader.extract_labware_definitions(
            protocol_source=protocol_source
        )
        for definition in labware_definitions:
            # Assume adding a labware definition is fast and there are not many labware
            # definitions, so we don't need to yield here.
            self._protocol_engine.add_labware_definition(definition)

        # fixme(mm, 2022-12-23): This does I/O and compute-bound parsing that will block
        # the event loop. Jira RSS-165.
        protocol = self._legacy_file_reader.read(protocol_source, labware_definitions)
        broker = None
        equipment_broker = None

        if protocol.api_level < LEGACY_PYTHON_API_VERSION_CUTOFF:
            broker = Broker()
            equipment_broker = EquipmentBroker[LegacyLoadInfo]()

            self._protocol_engine.add_plugin(
                LegacyContextPlugin(broker=broker, equipment_broker=equipment_broker)
            )

        context = self._legacy_context_creator.create(
            protocol=protocol,
            broker=broker,
            equipment_broker=equipment_broker,
        )

        self._task_queue.set_run_func(
            func=self._legacy_executor.execute,
            protocol=protocol,
            context=context,
        )

    def play(self) -> None:
        self._protocol_engine.play()

    def pause(self) -> None:
        self._protocol_engine.pause()

    async def stop(self) -> None:
        if self.was_started():
            await self._protocol_engine.stop()
        else:
            await self._protocol_engine.finish(
                drop_tips_and_home=False,
                set_run_status=False,
            )

    async def run(
        self,
        protocol_source: Optional[ProtocolSource] = None,
    ) -> ProtocolRunResult:
        # TODO(mc, 2022-01-11): move load to runner creation, remove from `run`
        # currently `protocol_source` arg is only used by tests
        if protocol_source:
            await self.load(protocol_source=protocol_source)

        await self._hardware_api.home()
        self.play()
        self._task_queue.start()
        await self._task_queue.join()

        run_data = self._protocol_engine.state_view.get_summary()
        commands = self._protocol_engine.state_view.commands.get_all()
        return ProtocolRunResult(commands=commands, state_summary=run_data)


class JsonRunner(AbstractRunner):
    """Protocol runner implementation for json protocols."""

    def __init__(
        self,
        protocol_engine: ProtocolEngine,
        hardware_api: HardwareControlAPI,
        task_queue: Optional[TaskQueue] = None,
        json_file_reader: Optional[JsonFileReader] = None,
        json_translator: Optional[JsonTranslator] = None,
    ) -> None:
        """Initialize the JsonRunner with its dependencies."""
        self._protocol_engine = protocol_engine
        self._hardware_api = hardware_api
        self._json_file_reader = json_file_reader or JsonFileReader()
        self._json_translator = json_translator or JsonTranslator()
        # TODO(mc, 2022-01-11): replace task queue with specific implementations
        # of runner interface
        self._task_queue = task_queue or TaskQueue(cleanup_func=protocol_engine.finish)

    def was_started(self) -> bool:
        return self._protocol_engine.state_view.commands.has_been_played()

    async def load(self, protocol_source: ProtocolSource) -> None:
        labware_definitions = await protocol_reader.extract_labware_definitions(
            protocol_source=protocol_source
        )
        for definition in labware_definitions:
            # Assume adding a labware definition is fast and there are not many labware
            # definitions, so we don't need to yield here.
            self._protocol_engine.add_labware_definition(definition)

        protocol = await anyio.to_thread.run_sync(
            self._json_file_reader.read,
            protocol_source,
        )

        commands = await anyio.to_thread.run_sync(
            self._json_translator.translate_commands,
            protocol,
        )

        # Add commands and liquids to the ProtocolEngine.
        #
        # We yield on every iteration so that loading large protocols doesn't block the
        # event loop. With a 24-step 10k-command protocol (See RQA-443), adding all the
        # commands can take 3 to 7 seconds.
        #
        # It wouldn't be safe to do this in a worker thread because each addition
        # invokes the ProtocolEngine's ChangeNotifier machinery, which is not
        # thread-safe.
        liquids = await anyio.to_thread.run_sync(
            self._json_translator.translate_liquids, protocol
        )
        for liquid in liquids:
            self._protocol_engine.add_liquid(
                id=liquid.id,
                name=liquid.displayName,
                description=liquid.description,
                color=liquid.displayColor,
            )
            await _yield()
        for command in commands:
            self._protocol_engine.add_command(request=command)
            await _yield()

        self._task_queue.set_run_func(func=self._protocol_engine.wait_until_complete)

    def play(self) -> None:
        self._protocol_engine.play()

    def pause(self) -> None:
        self._protocol_engine.pause()

    async def stop(self) -> None:
        if self.was_started():
            await self._protocol_engine.stop()
        else:
            await self._protocol_engine.finish(
                drop_tips_and_home=False,
                set_run_status=False,
            )

    async def run(
        self,
        protocol_source: Optional[ProtocolSource] = None,
    ) -> ProtocolRunResult:
        # TODO(mc, 2022-01-11): move load to runner creation, remove from `run`
        # currently `protocol_source` arg is only used by tests
        if protocol_source:
            await self.load(protocol_source)

        await self._hardware_api.home()
        self.play()
        self._task_queue.start()
        await self._task_queue.join()

        run_data = self._protocol_engine.state_view.get_summary()
        commands = self._protocol_engine.state_view.commands.get_all()
        return ProtocolRunResult(commands=commands, state_summary=run_data)


class LiveRunner(AbstractRunner):
    """Protocol runner implementation for setup commands."""

    def __init__(
        self,
        protocol_engine: ProtocolEngine,
        hardware_api: HardwareControlAPI,
    ) -> None:
        """Initialize the LiveRunner with its dependencies."""
        self._protocol_engine = protocol_engine
        # TODO(mc, 2022-01-11): replace task queue with specific implementations
        # of runner interface
        self._hardware_api = hardware_api

    def was_started(self) -> bool:
        return self._protocol_engine.state_view.commands.has_been_played()

    async def load(self, protocol_source: ProtocolSource) -> None:
        raise NotImplementedError(
            "LiveRunner.load() not supported for setup commands. no protocol associated."
        )

    def play(self) -> None:
        self._protocol_engine.play()

    def pause(self) -> None:
        self._protocol_engine.pause()

    async def stop(self) -> None:
        if self.was_started():
            await self._protocol_engine.stop()
        else:
            await self._protocol_engine.finish(
                drop_tips_and_home=False,
                set_run_status=False,
            )

    async def run(
        self,
        protocol_source: Optional[ProtocolSource] = None,
    ) -> ProtocolRunResult:
        if protocol_source:
            self.load(protocol_source)

        await self._hardware_api.home()

        run_data = self._protocol_engine.state_view.get_summary()
        commands = self._protocol_engine.state_view.commands.get_all()
        return ProtocolRunResult(commands=commands, state_summary=run_data)


def create_protocol_runner(
    protocol_config: Optional[Union[JsonProtocolConfig, PythonProtocolConfig]],
    protocol_engine: ProtocolEngine,
    hardware_api: HardwareControlAPI,
    task_queue: Optional[TaskQueue] = None,
    json_file_reader: Optional[JsonFileReader] = None,
    json_translator: Optional[JsonTranslator] = None,
    legacy_file_reader: Optional[LegacyFileReader] = None,
    legacy_context_creator: Optional[LegacyContextCreator] = None,
    legacy_executor: Optional[LegacyExecutor] = None,
) -> AbstractRunner:
    """Create a protocol runner."""
    if protocol_config:
        if (
            isinstance(protocol_config, JsonProtocolConfig)
            and protocol_config.schema_version >= LEGACY_JSON_SCHEMA_VERSION_CUTOFF
        ):
            return JsonRunner(
                protocol_engine=protocol_engine,
                hardware_api=hardware_api,
                json_file_reader=json_file_reader,
                json_translator=json_translator,
                task_queue=task_queue,
            )
        else:
            return PythonAndLegacyRunner(
                protocol_engine=protocol_engine,
                hardware_api=hardware_api,
                task_queue=task_queue,
                legacy_file_reader=legacy_file_reader,
                legacy_context_creator=legacy_context_creator,
                legacy_executor=legacy_executor,
            )

    return LiveRunner(
        protocol_engine=protocol_engine,
        hardware_api=hardware_api,
    )


async def _yield() -> None:
    """Yield execution to the event loop, giving other tasks a chance to run."""
    await asyncio.sleep(0)
