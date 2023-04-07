"""Tests for the EngineStore interface."""
from datetime import datetime
import inspect
import pytest
from decoy import Decoy, matchers

from opentrons_shared_data.robot.dev_types import RobotType

from opentrons.types import DeckSlotName
from opentrons.hardware_control import HardwareControlAPI
from opentrons.protocol_engine import (
    ProtocolEngine,
    StateSummary,
    protocol_engine_creator,
    types as pe_types,
    Config as ProtocolEngineConfig,
)
from opentrons.protocol_runner import ProtocolRunResult

from robot_server.maintenance_runs.engine_store import (
    MaintenanceEngineStore,
    EngineConflictError,
)


@pytest.fixture
def hardware_api(
    decoy: Decoy,
) -> HardwareControlAPI:
    """Return a mock in the shape of a HardwareControlAPI."""
    # TODO(mc, 2021-06-11): to make these test more effective and valuable, we
    # should pass in some sort of actual, valid HardwareAPI instead of a mock
    return decoy.mock(cls=HardwareControlAPI)


@pytest.fixture
def subject(decoy: Decoy, hardware_api: HardwareControlAPI) -> MaintenanceEngineStore:
    """Get a EngineStore test subject."""
    return MaintenanceEngineStore(
        hardware_api=hardware_api,
        # Arbitrary choice of robot_type. Tests where robot_type matters should
        # construct their own EngineStore.
        robot_type="OT-2 Standard",
    )


async def test_create_engine(subject: MaintenanceEngineStore) -> None:
    """It should create an engine for a run."""
    result = await subject.create(run_id="run-id", labware_offsets=[])

    assert subject.current_run_id == "run-id"
    assert isinstance(result, StateSummary)
    assert isinstance(subject.engine, ProtocolEngine)


@pytest.mark.parametrize("robot_type", ["OT-2 Standard", "OT-3 Standard"])
async def test_create_engine_uses_robot_type(
    decoy: Decoy, robot_type: RobotType
) -> None:
    """It should create ProtocolEngines with the given robot type."""
    # TODO(mc, 2021-06-11): to make these test more effective and valuable, we
    # should pass in some sort of actual, valid HardwareAPI instead of a mock
    hardware_api = decoy.mock(cls=HardwareControlAPI)
    subject = MaintenanceEngineStore(hardware_api=hardware_api, robot_type=robot_type)

    await subject.create(run_id="run-id", labware_offsets=[])

    assert subject.engine.state_view.config.robot_type == robot_type


async def test_create_engine_with_labware_offsets(subject: MaintenanceEngineStore) -> None:
    """It should create an engine for a run with labware offsets."""
    labware_offset = pe_types.LabwareOffsetCreate(
        definitionUri="namespace/load_name/version",
        location=pe_types.LabwareOffsetLocation(slotName=DeckSlotName.SLOT_5),
        vector=pe_types.LabwareOffsetVector(x=1, y=2, z=3),
    )

    result = await subject.create(
        run_id="run-id",
        labware_offsets=[labware_offset],
    )

    assert result.labwareOffsets == [
        pe_types.LabwareOffset.construct(
            id=matchers.IsA(str),
            createdAt=matchers.IsA(datetime),
            definitionUri="namespace/load_name/version",
            location=pe_types.LabwareOffsetLocation(slotName=DeckSlotName.SLOT_5),
            vector=pe_types.LabwareOffsetVector(x=1, y=2, z=3),
        )
    ]


async def test_archives_state_if_engine_already_exists(subject: MaintenanceEngineStore) -> None:
    """It should not create more than one engine."""
    await subject.create(run_id="run-id-1", labware_offsets=[])

    with pytest.raises(EngineConflictError):
        await subject.create(run_id="run-id-2", labware_offsets=[])

    assert subject.current_run_id == "run-id-1"


async def test_clear_engine(
        decoy: Decoy,
        monkeypatch: pytest.MonkeyPatch,
        hardware_api: HardwareControlAPI,
) -> None:
    """It should clear a stored engine entry."""
    mock_engine_creator = decoy.mock(func=protocol_engine_creator.create_protocol_engine)
    monkeypatch.setattr(
        protocol_engine_creator, "create_protocol_engine", mock_engine_creator)
    mock_engine = decoy.mock(cls=ProtocolEngine)
    subject = MaintenanceEngineStore(
        hardware_api=hardware_api,
        # Arbitrary choice of robot_type. Tests where robot_type matters should
        # construct their own EngineStore.
        robot_type="OT-2 Standard",
    )
    decoy.when(await protocol_engine_creator.create_protocol_engine(
        hardware_api=hardware_api,
        config=ProtocolEngineConfig(robot_type="OT-2 Standard",
                                    block_on_door_open=False)
    )).then_return(mock_engine)
    decoy.when(mock_engine.state_view.commands.get_is_okay_to_clear()).then_return(
        True)
    await subject.create(run_id="run-id", labware_offsets=[])
    result = await subject.clear()

    assert subject.current_run_id is None
    assert isinstance(result, ProtocolRunResult)

    with pytest.raises(AssertionError):
        subject.engine


async def test_clear_running_engine(
        decoy: Decoy,
        monkeypatch: pytest.MonkeyPatch,
        hardware_api: HardwareControlAPI,
        subject: MaintenanceEngineStore,
) -> None:
    """It should raise a conflict if the engine is not stopped."""
    mock_engine_creator = decoy.mock(
        func=protocol_engine_creator.create_protocol_engine)
    monkeypatch.setattr(
        protocol_engine_creator, "create_protocol_engine", mock_engine_creator)
    mock_engine = decoy.mock(cls=ProtocolEngine)
    subject = MaintenanceEngineStore(
        hardware_api=hardware_api,
        # Arbitrary choice of robot_type. Tests where robot_type matters should
        # construct their own EngineStore.
        robot_type="OT-2 Standard",
    )
    decoy.when(await protocol_engine_creator.create_protocol_engine(
        hardware_api=hardware_api,
        config=ProtocolEngineConfig(robot_type="OT-2 Standard",
                                    block_on_door_open=False)
    )).then_return(mock_engine)
    decoy.when(mock_engine.state_view.commands.get_is_okay_to_clear()).then_return(False)
    await subject.create(run_id="run-id", labware_offsets=[])

    with pytest.raises(EngineConflictError):
        await subject.clear()
