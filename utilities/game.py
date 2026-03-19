import wpilib
from wpilib import SendableChooser, SmartDashboard as _SD


# This will default to the blue alliance if a proper link to the driver station has not yet been established
def is_red() -> bool:
    return wpilib.DriverStation.getAlliance() == wpilib.DriverStation.Alliance.kRed


_side_chooser: SendableChooser | None = None


def init_side_chooser() -> None:
    global _side_chooser
    _side_chooser = SendableChooser()
    _side_chooser.setDefaultOption("Right", False)
    _side_chooser.addOption("Left", True)
    _SD.putData("Field Side", _side_chooser)


def is_left() -> bool:
    if _side_chooser is None:
        return False
    return bool(_side_chooser.getSelected())


def is_match() -> bool:
    return wpilib.DriverStation.isFMSAttached()


def is_sim() -> bool:
    return wpilib.RobotBase.isSimulation()


def is_auton() -> bool:
    mode = wpilib.SmartDashboard.getString('/robot/mode', '')
    return mode in ['auto']


def is_disabled() -> bool:
    mode = wpilib.SmartDashboard.getString('/robot/mode', '')
    return mode in ['disabled', '']


# ── REBUILT 2026 Shift Timing Constants (seconds remaining in teleop) ─────────
_TRANSITION_END_S = 125.0   # Shift 1 begins
_SHIFT_2_START_S  = 100.0
_SHIFT_3_START_S  =  75.0
_SHIFT_4_START_S  =  50.0
_ENDGAME_START_S  =  25.0
_SHOOT_BUFFER_S   =   1.0   # pre/post buffer for indicator


def _shift_number(match_time: float) -> int:
    """Return 0 = both active, 1-4 = shift number."""
    if match_time > _TRANSITION_END_S or match_time <= _ENDGAME_START_S:
        return 0
    if match_time > _SHIFT_2_START_S:
        return 1
    if match_time > _SHIFT_3_START_S:
        return 2
    if match_time > _SHIFT_4_START_S:
        return 3
    return 4


def _phase_time_remaining(match_time: float) -> float:
    """Seconds left until the current shift ends (0 during transition/endgame)."""
    for boundary in (_TRANSITION_END_S, _SHIFT_2_START_S, _SHIFT_3_START_S,
                     _SHIFT_4_START_S, _ENDGAME_START_S):
        if match_time > boundary:
            return match_time - boundary
    return 0.0


def _hub_active_at(match_time: float, game_msg: str) -> bool:
    """Is our alliance's HUB active at this exact match time?"""
    shift = _shift_number(match_time)
    if shift == 0:      # Transition or End Game — both HUBs active
        return True
    if not game_msg:    # No FMS data yet — assume active (safe default)
        return True
    # game_msg is 'R' or 'B': the alliance whose HUB goes inactive FIRST
    we_go_inactive_first = (game_msg == 'R') == is_red()
    # Odd shifts deactivate the alliance named by game_msg; even shifts flip
    our_hub_inactive = (shift % 2 == 1) == we_go_inactive_first
    return not our_hub_inactive


def hub_shoot_indicator() -> tuple[bool, float]:
    """Return (can_shoot, phase_seconds_remaining).

    can_shoot is True when our HUB is active, extended by _SHOOT_BUFFER_S on
    each side so the indicator lights 1 s before the HUB activates (shot
    flight time) and stays lit 1 s after it deactivates (grace period).
    phase_seconds_remaining counts down to the end of the current shift.
    """
    match_time = wpilib.DriverStation.getMatchTime()
    if match_time < 0:
        match_time = 0.0
    game_msg = wpilib.DriverStation.getGameSpecificMessage()

    can_shoot = (
        _hub_active_at(match_time, game_msg)
        or _hub_active_at(match_time - _SHOOT_BUFFER_S, game_msg)  # pre-buffer
        or _hub_active_at(match_time + _SHOOT_BUFFER_S, game_msg)  # post-buffer
    )
    return can_shoot, _phase_time_remaining(match_time)
