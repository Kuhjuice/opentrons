"""Tests for RunDataManager."""
from typing import Optional

import pytest
from datetime import datetime
from decoy import Decoy, matchers

from opentrons.types import DeckSlotName
from opentrons.protocol_runner import ProtocolRunResult
from opentrons.protocol_engine import (
    EngineStatus,
    StateSummary,
    commands,
    types as pe_types,
    CommandSlice,
    CurrentCommand,
    ErrorOccurrence,
    LoadedLabware,
    LoadedPipette,
    LoadedModule,
    LabwareOffset,
)

from robot_server.protocols import ProtocolResource
from robot_server.maintenance_run.engine_store import EngineStore, EngineConflictError
from robot_server.maintenance_run.maintenance_run_data_manager import (
    MaintenanceRunDataManager,
    RunNotCurrentError,
)
from robot_server.maintenance_run.maintenance_run_models import MaintenanceRun

from robot_server.service.task_runner import TaskRunner

from opentrons.protocol_engine import Liquid


@pytest.fixture
def mock_engine_store(decoy: Decoy) -> EngineStore:
    """Get a mock EngineStore."""
    mock = decoy.mock(cls=EngineStore)
    decoy.when(mock.current_run_id).then_return(None)
    return mock


@pytest.fixture
def engine_state_summary() -> StateSummary:
    """Get a StateSummary value object."""
    return StateSummary(
        status=EngineStatus.IDLE,
        errors=[ErrorOccurrence.construct(id="some-error-id")],  # type: ignore[call-arg]
        labware=[LoadedLabware.construct(id="some-labware-id")],  # type: ignore[call-arg]
        labwareOffsets=[LabwareOffset.construct(id="some-labware-offset-id")],  # type: ignore[call-arg]
        pipettes=[LoadedPipette.construct(id="some-pipette-id")],  # type: ignore[call-arg]
        modules=[LoadedModule.construct(id="some-module-id")],  # type: ignore[call-arg]
        liquids=[Liquid(id="some-liquid-id", displayName="liquid", description="desc")],
    )


@pytest.fixture
def run_command() -> commands.Command:
    """Get a ProtocolEngine Command value object."""
    return commands.WaitForResume(
        id="command-id",
        key="command-key",
        createdAt=datetime(year=2021, month=1, day=1),
        status=commands.CommandStatus.SUCCEEDED,
        params=commands.WaitForResumeParams(message="Hello"),
    )


@pytest.fixture
def subject(
    mock_engine_store: EngineStore,
) -> MaintenanceRunDataManager:
    """Get a MaintenanceRunDataManager test subject."""
    return MaintenanceRunDataManager(
        engine_store=mock_engine_store,
    )


async def test_create(
    decoy: Decoy,
    mock_engine_store: EngineStore,
    subject: MaintenanceRunDataManager,
    engine_state_summary: StateSummary,
) -> None:
    """It should create an engine and a persisted run resource."""
    run_id = "hello world"
    created_at = datetime(year=2021, month=1, day=1)

    decoy.when(
        await mock_engine_store.create(run_id=run_id, labware_offsets=[], protocol=None)
    ).then_return(engine_state_summary)

    result = await subject.create(
        run_id=run_id,
        created_at=created_at,
        labware_offsets=[],
    )

    assert result == MaintenanceRun(
        id=run_id,
        createdAt=created_at,
        current=True,
        status=engine_state_summary.status,
        errors=engine_state_summary.errors,
        labware=engine_state_summary.labware,
        labwareOffsets=engine_state_summary.labwareOffsets,
        pipettes=engine_state_summary.pipettes,
        modules=engine_state_summary.modules,
        liquids=engine_state_summary.liquids,
    )


async def test_create_with_options(
    decoy: Decoy,
    mock_engine_store: EngineStore,
    subject: MaintenanceRunDataManager,
    engine_state_summary: StateSummary,
) -> None:
    """It should handle creation with a protocol and labware offsets."""
    run_id = "hello world"
    created_at = datetime(year=2021, month=1, day=1)

    protocol = ProtocolResource(
        protocol_id="protocol-id",
        created_at=datetime(year=2022, month=2, day=2),
        source=None,  # type: ignore[arg-type]
        protocol_key=None,
    )

    labware_offset = pe_types.LabwareOffsetCreate(
        definitionUri="namespace/load_name/version",
        location=pe_types.LabwareOffsetLocation(slotName=DeckSlotName.SLOT_5),
        vector=pe_types.LabwareOffsetVector(x=1, y=2, z=3),
    )

    decoy.when(
        await mock_engine_store.create(
            run_id=run_id,
            labware_offsets=[labware_offset],
            protocol=None,
        )
    ).then_return(engine_state_summary)

    result = await subject.create(
        run_id=run_id,
        created_at=created_at,
        labware_offsets=[labware_offset],
    )

    assert result == MaintenanceRun(
        id=run_id,
        createdAt=created_at,
        current=True,
        status=engine_state_summary.status,
        errors=engine_state_summary.errors,
        labware=engine_state_summary.labware,
        labwareOffsets=engine_state_summary.labwareOffsets,
        pipettes=engine_state_summary.pipettes,
        modules=engine_state_summary.modules,
        liquids=engine_state_summary.liquids,
    )


async def test_create_engine_error(
    decoy: Decoy,
    mock_engine_store: EngineStore,
    subject: MaintenanceRunDataManager,
) -> None:
    """It should not create a resource if engine creation fails."""
    run_id = "hello world"
    created_at = datetime(year=2021, month=1, day=1)

    decoy.when(
        await mock_engine_store.create(run_id, labware_offsets=[], protocol=None)
    ).then_raise(EngineConflictError("oh no"))

    with pytest.raises(EngineConflictError):
        await subject.create(
            run_id=run_id,
            created_at=created_at,
            labware_offsets=[],
        )


# async def test_get_current_run(
#     decoy: Decoy,
#     mock_engine_store: EngineStore,
#     mock_run_store: RunStore,
#     subject: RunDataManager,
#     engine_state_summary: StateSummary,
#     run_resource: RunResource,
# ) -> None:
#     """It should get the current run from the engine."""
#     run_id = "hello world"
#
#     decoy.when(mock_run_store.get(run_id=run_id)).then_return(run_resource)
#     decoy.when(mock_engine_store.current_run_id).then_return(run_id)
#     decoy.when(mock_engine_store.engine.state_view.get_summary()).then_return(
#         engine_state_summary
#     )
#
#     result = subject.get(run_id=run_id)
#
#     assert result == Run(
#         current=True,
#         id=run_resource.run_id,
#         protocolId=run_resource.protocol_id,
#         createdAt=run_resource.created_at,
#         actions=run_resource.actions,
#         status=engine_state_summary.status,
#         errors=engine_state_summary.errors,
#         labware=engine_state_summary.labware,
#         labwareOffsets=engine_state_summary.labwareOffsets,
#         pipettes=engine_state_summary.pipettes,
#         modules=engine_state_summary.modules,
#         liquids=engine_state_summary.liquids,
#     )
#     assert subject.current_run_id == run_id


# async def test_get_historical_run(
#     decoy: Decoy,
#     mock_engine_store: EngineStore,
#     mock_run_store: RunStore,
#     subject: RunDataManager,
#     engine_state_summary: StateSummary,
#     run_resource: RunResource,
# ) -> None:
#     """It should get a historical run from the store."""
#     run_id = "hello world"
#
#     decoy.when(mock_run_store.get(run_id=run_id)).then_return(run_resource)
#     decoy.when(mock_run_store.get_state_summary(run_id=run_id)).then_return(
#         engine_state_summary
#     )
#     decoy.when(mock_engine_store.current_run_id).then_return("some other id")
#
#     result = subject.get(run_id=run_id)
#
#     assert result == Run(
#         current=False,
#         id=run_resource.run_id,
#         protocolId=run_resource.protocol_id,
#         createdAt=run_resource.created_at,
#         actions=run_resource.actions,
#         status=engine_state_summary.status,
#         errors=engine_state_summary.errors,
#         labware=engine_state_summary.labware,
#         labwareOffsets=engine_state_summary.labwareOffsets,
#         pipettes=engine_state_summary.pipettes,
#         modules=engine_state_summary.modules,
#         liquids=engine_state_summary.liquids,
#     )


# async def test_get_historical_run_no_data(
#     decoy: Decoy,
#     mock_engine_store: EngineStore,
#     mock_run_store: RunStore,
#     subject: RunDataManager,
#     run_resource: RunResource,
# ) -> None:
#     """It should get a historical run from the store."""
#     run_id = "hello world"
#
#     decoy.when(mock_run_store.get(run_id=run_id)).then_return(run_resource)
#     decoy.when(mock_run_store.get_state_summary(run_id=run_id)).then_return(None)
#     decoy.when(mock_engine_store.current_run_id).then_return("some other id")
#
#     result = subject.get(run_id=run_id)
#
#     assert result == Run(
#         current=False,
#         id=run_resource.run_id,
#         protocolId=run_resource.protocol_id,
#         createdAt=run_resource.created_at,
#         actions=run_resource.actions,
#         status=EngineStatus.STOPPED,
#         errors=[],
#         labware=[],
#         labwareOffsets=[],
#         pipettes=[],
#         modules=[],
#         liquids=[],
#     )


# async def test_get_all_runs(
#     decoy: Decoy,
#     mock_engine_store: EngineStore,
#     mock_run_store: RunStore,
#     subject: RunDataManager,
# ) -> None:
#     """It should get all runs, including current and historical."""
#     current_run_data = StateSummary(
#         status=EngineStatus.IDLE,
#         errors=[ErrorOccurrence.construct(id="current-error-id")],  # type: ignore[call-arg]
#         labware=[LoadedLabware.construct(id="current-labware-id")],  # type: ignore[call-arg]
#         labwareOffsets=[LabwareOffset.construct(id="current-labware-offset-id")],  # type: ignore[call-arg]
#         pipettes=[LoadedPipette.construct(id="current-pipette-id")],  # type: ignore[call-arg]
#         modules=[LoadedModule.construct(id="current-module-id")],  # type: ignore[call-arg]
#         liquids=[Liquid(id="some-liquid-id", displayName="liquid", description="desc")],
#     )
#
#     historical_run_data = StateSummary(
#         status=EngineStatus.STOPPED,
#         errors=[ErrorOccurrence.construct(id="old-error-id")],  # type: ignore[call-arg]
#         labware=[LoadedLabware.construct(id="old-labware-id")],  # type: ignore[call-arg]
#         labwareOffsets=[LabwareOffset.construct(id="old-labware-offset-id")],  # type: ignore[call-arg]
#         pipettes=[LoadedPipette.construct(id="old-pipette-id")],  # type: ignore[call-arg]
#         modules=[LoadedModule.construct(id="old-module-id")],  # type: ignore[call-arg]
#         liquids=[],
#     )
#
#     current_run_resource = RunResource(
#         run_id="current-run",
#         protocol_id=None,
#         created_at=datetime(year=2022, month=2, day=2),
#         actions=[],
#     )
#
#     historical_run_resource = RunResource(
#         run_id="historical-run",
#         protocol_id=None,
#         created_at=datetime(year=2023, month=3, day=3),
#         actions=[],
#     )
#
#     decoy.when(mock_engine_store.current_run_id).then_return("current-run")
#     decoy.when(mock_engine_store.engine.state_view.get_summary()).then_return(
#         current_run_data
#     )
#     decoy.when(mock_run_store.get_state_summary("historical-run")).then_return(
#         historical_run_data
#     )
#     decoy.when(mock_run_store.get_all()).then_return(
#         [historical_run_resource, current_run_resource]
#     )
#
#     result = subject.get_all()
#
#     assert result == [
#         Run(
#             current=False,
#             id=historical_run_resource.run_id,
#             protocolId=historical_run_resource.protocol_id,
#             createdAt=historical_run_resource.created_at,
#             actions=historical_run_resource.actions,
#             status=historical_run_data.status,
#             errors=historical_run_data.errors,
#             labware=historical_run_data.labware,
#             labwareOffsets=historical_run_data.labwareOffsets,
#             pipettes=historical_run_data.pipettes,
#             modules=historical_run_data.modules,
#             liquids=historical_run_data.liquids,
#         ),
#         Run(
#             current=True,
#             id=current_run_resource.run_id,
#             protocolId=current_run_resource.protocol_id,
#             createdAt=current_run_resource.created_at,
#             actions=current_run_resource.actions,
#             status=current_run_data.status,
#             errors=current_run_data.errors,
#             labware=current_run_data.labware,
#             labwareOffsets=current_run_data.labwareOffsets,
#             pipettes=current_run_data.pipettes,
#             modules=current_run_data.modules,
#             liquids=current_run_data.liquids,
#         ),
#     ]


# async def test_delete_current_run(
#     decoy: Decoy,
#     mock_engine_store: EngineStore,
#     mock_run_store: RunStore,
#     subject: RunDataManager,
# ) -> None:
#     """It should delete the current run from the engine."""
#     run_id = "hello world"
#     decoy.when(mock_engine_store.current_run_id).then_return(run_id)
#
#     await subject.delete(run_id=run_id)
#
#     decoy.verify(
#         await mock_engine_store.clear(),
#         mock_run_store.remove(run_id=run_id),
#     )


# async def test_delete_historical_run(
#     decoy: Decoy,
#     mock_engine_store: EngineStore,
#     mock_run_store: RunStore,
#     subject: RunDataManager,
# ) -> None:
#     """It should delete a historical run from the store."""
#     run_id = "hello world"
#     decoy.when(mock_engine_store.current_run_id).then_return("some other id")
#
#     await subject.delete(run_id=run_id)
#
#     decoy.verify(await mock_engine_store.clear(), times=0)
#     decoy.verify(mock_run_store.remove(run_id=run_id), times=1)


# async def test_update_current(
#     decoy: Decoy,
#     engine_state_summary: StateSummary,
#     run_resource: RunResource,
#     run_command: commands.Command,
#     mock_engine_store: EngineStore,
#     mock_run_store: RunStore,
#     subject: RunDataManager,
# ) -> None:
#     """It should persist the current run and clear the engine on current=false."""
#     run_id = "hello world"
#     decoy.when(mock_engine_store.current_run_id).then_return(run_id)
#     decoy.when(await mock_engine_store.clear()).then_return(
#         ProtocolRunResult(commands=[run_command], state_summary=engine_state_summary)
#     )
#
#     decoy.when(
#         mock_run_store.update_run_state(
#             run_id=run_id,
#             summary=engine_state_summary,
#             commands=[run_command],
#         )
#     ).then_return(run_resource)
#
#     result = await subject.update(run_id=run_id, current=False)
#
#     assert result == Run(
#         current=False,
#         id=run_resource.run_id,
#         protocolId=run_resource.protocol_id,
#         createdAt=run_resource.created_at,
#         actions=run_resource.actions,
#         status=engine_state_summary.status,
#         errors=engine_state_summary.errors,
#         labware=engine_state_summary.labware,
#         labwareOffsets=engine_state_summary.labwareOffsets,
#         pipettes=engine_state_summary.pipettes,
#         modules=engine_state_summary.modules,
#         liquids=engine_state_summary.liquids,
#     )


# @pytest.mark.parametrize("current", [None, True])
# async def test_update_current_noop(
#     decoy: Decoy,
#     engine_state_summary: StateSummary,
#     run_resource: RunResource,
#     run_command: commands.Command,
#     mock_engine_store: EngineStore,
#     mock_run_store: RunStore,
#     subject: RunDataManager,
#     current: Optional[bool],
# ) -> None:
#     """It should noop on current=None and current=True."""
#     run_id = "hello world"
#     decoy.when(mock_engine_store.current_run_id).then_return(run_id)
#     decoy.when(mock_engine_store.engine.state_view.get_summary()).then_return(
#         engine_state_summary
#     )
#     decoy.when(mock_run_store.get(run_id=run_id)).then_return(run_resource)
#
#     result = await subject.update(run_id=run_id, current=current)
#
#     decoy.verify(await mock_engine_store.clear(), times=0)
#     decoy.verify(
#         mock_run_store.update_run_state(
#             run_id=run_id,
#             summary=matchers.Anything(),
#             commands=matchers.Anything(),
#         ),
#         times=0,
#     )
#
#     assert result == Run(
#         current=True,
#         id=run_resource.run_id,
#         protocolId=run_resource.protocol_id,
#         createdAt=run_resource.created_at,
#         actions=run_resource.actions,
#         status=engine_state_summary.status,
#         errors=engine_state_summary.errors,
#         labware=engine_state_summary.labware,
#         labwareOffsets=engine_state_summary.labwareOffsets,
#         pipettes=engine_state_summary.pipettes,
#         modules=engine_state_summary.modules,
#         liquids=engine_state_summary.liquids,
#     )


# async def test_update_current_not_allowed(
#     decoy: Decoy,
#     engine_state_summary: StateSummary,
#     run_resource: RunResource,
#     run_command: commands.Command,
#     mock_engine_store: EngineStore,
#     mock_run_store: RunStore,
#     subject: RunDataManager,
# ) -> None:
#     """It should noop on current=None."""
#     run_id = "hello world"
#     decoy.when(mock_engine_store.current_run_id).then_return("some other id")
#
#     with pytest.raises(RunNotCurrentError):
#         await subject.update(run_id=run_id, current=False)


# def test_get_commands_slice_current_run(
#     decoy: Decoy,
#     subject: RunDataManager,
#     mock_engine_store: EngineStore,
#     run_command: commands.Command,
# ) -> None:
#     """Should get a sliced command list from engine store."""
#     expected_commands_result = [
#         commands.WaitForResume(
#             id="command-id-2",
#             key="command-key",
#             createdAt=datetime(year=2021, month=1, day=1),
#             status=commands.CommandStatus.SUCCEEDED,
#             params=commands.WaitForResumeParams(message="Hello"),
#         ),
#         run_command,
#     ]
#
#     expected_command_slice = CommandSlice(
#         commands=expected_commands_result, cursor=1, total_length=3
#     )
#     decoy.when(mock_engine_store.current_run_id).then_return("run-id")
#     decoy.when(
#         mock_engine_store.engine.state_view.commands.get_slice(1, 2)
#     ).then_return(expected_command_slice)
#
#     result = subject.get_commands_slice("run-id", 1, 2)
#
#     assert expected_command_slice == result
#
