"""In-memory storage of ProtocolEngine instances."""
from typing import List, NamedTuple, Optional

from opentrons_shared_data.robot.dev_types import RobotType

from opentrons.config import feature_flags
from opentrons.hardware_control import HardwareControlAPI
from opentrons.protocol_runner import ProtocolRunResult
from opentrons.protocol_engine import (
    ProtocolEngine,
    Config as ProtocolEngineConfig,
    StateSummary,
    LabwareOffsetCreate,
    protocol_engine_creator,
)


class EngineConflictError(RuntimeError):
    """An error raised if an active engine is already initialized.

    The store will not create a new engine unless the "current" runner/engine
    pair is idle.
    """


class EngineByRunId(NamedTuple):
    """A stored ProtocolRunner/ProtocolEngine pair."""

    run_id: str
    engine: ProtocolEngine


class MaintenanceEngineStore:
    """Factory and in-memory storage for ProtocolEngine."""

    def __init__(
        self,
        hardware_api: HardwareControlAPI,
        robot_type: RobotType,
    ) -> None:
        """Initialize an engine storage interface.

        Arguments:
            hardware_api: Hardware control API instance used for ProtocolEngine
                construction.
            robot_type: Passed along to `opentrons.protocol_engine.Config`.
        """
        self._hardware_api = hardware_api
        self._robot_type = robot_type
        self._run_id: Optional[str] = None
        self._engine: Optional[ProtocolEngine] = None

    @property
    def engine(self) -> ProtocolEngine:
        """Get the "current" ProtocolEngine."""
        assert self._engine is not None, "Engine not yet created."
        return self._engine

    @property
    def current_run_id(self) -> Optional[str]:
        """Get the run identifier associated with the current engine/runner pair."""
        return self._run_id

    async def create(
        self,
        run_id: str,
        labware_offsets: List[LabwareOffsetCreate],
    ) -> StateSummary:
        """Create a ProtocolEngine for a given Run.

        Args:
            run_id: The run resource the engine is assigned to.
            labware_offsets: Labware offsets to create the engine with.

        Returns:
            The initial equipment and status summary of the engine.

        Raises:
            EngineConflictError: The current engine is not idle, so
            a new one may not be created.
        """
        if self._engine is not None:
            raise EngineConflictError(
                "Another maintenance run engine is currently active.")
        self._run_id = run_id
        print(self._hardware_api)
        print(self._robot_type)
        print(feature_flags.enable_door_safety_switch())
        self._engine = await protocol_engine_creator.create_protocol_engine(
            hardware_api=self._hardware_api,
            config=ProtocolEngineConfig(
                robot_type=self._robot_type,
                block_on_door_open=feature_flags.enable_door_safety_switch(),
            ),
        )
        for offset in labware_offsets:
            self._engine.add_labware_offset(offset)

        return self._engine.state_view.get_summary()

    async def clear(self) -> ProtocolRunResult:
        """Remove the ProtocolEngine.

        Raises:
            EngineConflictError: The current runner/engine pair is not idle, so
            they cannot be cleared.
        """
        state_view = self.engine.state_view

        if state_view.commands.get_is_okay_to_clear():
            await self.engine.finish(drop_tips_and_home=False, set_run_status=False)
        else:
            raise EngineConflictError("Current run is not idle or stopped.")

        run_data = state_view.get_summary()
        commands = state_view.commands.get_all()
        self._engine = None
        self._run_id = None
        return ProtocolRunResult(state_summary=run_data, commands=commands)
