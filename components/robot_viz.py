"""Side-profile Mechanism2d visualization of the robot.

Shows the intake arm, rollers, kicker, shooter flywheel, and hood
in a single 2D side-view that updates in real time during sim or
on the real robot via NetworkTables / AdvantageScope / Glass.
"""

import wpilib

from components.intake import IntakeComponent
from components.kicker import KickerComponent
from components.shooter import ShooterComponent


# ── Layout constants (meters, side-view with front=right) ────────────────

# Canvas size
CANVAS_W = 1.2
CANVAS_H = 1.0

# Robot body rectangle (visual reference)
BODY_X = 0.20
BODY_W = 0.70
BODY_Y = 0.20
BODY_H = 0.30  # body from y=0.20 to y=0.50

# Intake arm pivot — near the front-top of the body
PIVOT_X = BODY_X + BODY_W - 0.05  # slightly inset from front edge
PIVOT_Y = BODY_Y + BODY_H  # top of body

# Intake arm length (center of robot to ~4" past bumper)
ARM_LENGTH = 0.46  # ~18 inches

# Encoder mapping: 0.00 rot → arm forward/down, 0.25 rot → straight up
# In Mechanism2d, angle 0° = right (forward), 90° = up, CCW positive.
# Since encoder 0.00 = "down" meaning horizontal-forward, and 0.25 = up:
#   mechanism_angle = encoder_position * 360
# But "down" for the intake means it's reaching toward the ground, so
# at position 0 the arm should point forward and slightly downward.
# We'll map: angle = encoder_pos * 360 - 10  (slight offset so 0.00 is
# nearly horizontal, pointing toward the ground in front of the robot)
ARM_ANGLE_OFFSET = -10  # degrees below horizontal at position 0.00

# Internal path positions (along the bottom of the robot)
PATH_Y = BODY_Y + 0.05  # slightly above bottom of body
KICKER_X = 0.55
SHOOTER_X = 0.38
HOOD_X = 0.25

# Indicator sizes
INDICATOR_LEN = 0.08
ROLLER_LEN = 0.06


class RobotVisualization:
    """Mechanism2d side-profile of the robot for dashboard display."""

    intake: IntakeComponent
    kicker: KickerComponent
    shooter: ShooterComponent

    def __init__(self):
        self.mech = wpilib.Mechanism2d(CANVAS_W, CANVAS_H, wpilib.Color8Bit(30, 30, 30))

        # ── Robot body outline (static structural ligaments) ─────────
        body_root = self.mech.getRoot("body_bl", BODY_X, BODY_Y)
        # Bottom edge
        self.body_bottom = body_root.appendLigament(
            "bottom", BODY_W, 0, lineWidth=2,
            color=wpilib.Color8Bit(80, 80, 80),
        )
        # Front wall (right side) — upward from bottom-right
        body_fr = self.mech.getRoot("body_fr", BODY_X + BODY_W, BODY_Y)
        self.body_front = body_fr.appendLigament(
            "front", BODY_H, 90, lineWidth=2,
            color=wpilib.Color8Bit(80, 80, 80),
        )
        # Back wall (left side) — upward from bottom-left
        body_bl = self.mech.getRoot("body_bl_wall", BODY_X, BODY_Y)
        self.body_back = body_bl.appendLigament(
            "back", BODY_H, 90, lineWidth=2,
            color=wpilib.Color8Bit(80, 80, 80),
        )
        # Top edge
        body_tl = self.mech.getRoot("body_tl", BODY_X, BODY_Y + BODY_H)
        self.body_top = body_tl.appendLigament(
            "top", BODY_W, 0, lineWidth=2,
            color=wpilib.Color8Bit(80, 80, 80),
        )

        # ── Intake arm ───────────────────────────────────────────────
        arm_root = self.mech.getRoot("intake_pivot", PIVOT_X, PIVOT_Y)
        # Pivot dot (tiny ligament as visual anchor)
        arm_root.appendLigament(
            "pivot_dot", 0.02, 0, lineWidth=8,
            color=wpilib.Color8Bit(255, 255, 255),
        )

        self.arm = arm_root.appendLigament(
            "intake_arm", ARM_LENGTH, ARM_ANGLE_OFFSET,
            lineWidth=6,
            color=wpilib.Color8Bit(220, 140, 40),  # orange
        )

        # Roller indicator at arm tip — rotates when rollers spin
        self.roller_indicator = self.arm.appendLigament(
            "roller", ROLLER_LEN, 0,
            lineWidth=10,
            color=wpilib.Color8Bit(60, 60, 60),  # gray when off
        )
        # Sushi indicator — perpendicular to roller
        self.sushi_indicator = self.arm.appendLigament(
            "sushi", ROLLER_LEN, 90,
            lineWidth=8,
            color=wpilib.Color8Bit(60, 60, 60),
        )

        # ── Kicker indicator ─────────────────────────────────────────
        kicker_root = self.mech.getRoot("kicker", KICKER_X, PATH_Y)
        self.kicker_indicator = kicker_root.appendLigament(
            "kicker_wheel", INDICATOR_LEN, 0,
            lineWidth=8,
            color=wpilib.Color8Bit(60, 60, 60),
        )
        # Label ligament (static tiny line for visual anchor)
        kicker_root.appendLigament(
            "kicker_base", 0.02, 270, lineWidth=4,
            color=wpilib.Color8Bit(100, 100, 255),
        )

        # ── Shooter flywheel indicator ───────────────────────────────
        shooter_root = self.mech.getRoot("shooter", SHOOTER_X, PATH_Y)
        self.flywheel_indicator = shooter_root.appendLigament(
            "flywheel", INDICATOR_LEN + 0.02, 0,
            lineWidth=10,
            color=wpilib.Color8Bit(60, 60, 60),
        )
        shooter_root.appendLigament(
            "shooter_base", 0.02, 270, lineWidth=4,
            color=wpilib.Color8Bit(255, 60, 60),
        )

        # ── Hood indicator ───────────────────────────────────────────
        hood_root = self.mech.getRoot("hood", HOOD_X, PATH_Y + 0.05)
        self.hood = hood_root.appendLigament(
            "hood_flap", 0.12, 135,  # angled up-and-back by default
            lineWidth=5,
            color=wpilib.Color8Bit(180, 180, 180),
        )

        # ── Exit path arrow (static, shows ball exit direction) ──────
        exit_root = self.mech.getRoot("exit", BODY_X - 0.02, PATH_Y + 0.08)
        exit_root.appendLigament(
            "exit_arrow", 0.10, 160, lineWidth=3,
            color=wpilib.Color8Bit(255, 255, 100),
        )

        # ── Internal roller path (static dotted line) ────────────────
        path_root = self.mech.getRoot("path_start", KICKER_X + 0.05, PATH_Y)
        path_root.appendLigament(
            "path", KICKER_X - HOOD_X + 0.10, 180, lineWidth=2,
            color=wpilib.Color8Bit(60, 60, 80),
        )

        # Accumulated rotation for spinning indicators
        self._roller_angle = 0.0
        self._sushi_angle = 0.0
        self._kicker_angle = 0.0
        self._flywheel_angle = 0.0

    def setup(self):
        wpilib.SmartDashboard.putData("Robot Side View", self.mech)

    def execute(self):
        return
        # ── Intake arm angle ─────────────────────────────────────────
        enc_pos = self.intake.rotate_encoder.get_position().value
        arm_angle = enc_pos * 360 + ARM_ANGLE_OFFSET
        self.arm.setAngle(arm_angle)

        # ── Roller / sushi spinning indicators ───────────────────────
        # Rollers use DutyCycleOut so we check the target speed.
        # A spinning indicator accumulates angle each cycle.
        roller_active = abs(self.intake.target_intake_speed) > 0.01
        sushi_active = abs(self.intake.target_sushi_speed) > 0.01

        if roller_active:
            self._roller_angle += 15  # degrees per cycle
            self.roller_indicator.setColor(wpilib.Color8Bit(50, 220, 50))  # green
        else:
            self.roller_indicator.setColor(wpilib.Color8Bit(60, 60, 60))
        self.roller_indicator.setAngle(self._roller_angle % 360)

        if sushi_active:
            self._sushi_angle += 12
            self.sushi_indicator.setColor(wpilib.Color8Bit(50, 180, 220))  # cyan
        else:
            self.sushi_indicator.setColor(wpilib.Color8Bit(60, 60, 60))
        self.sushi_indicator.setAngle(90 + (self._sushi_angle % 360))

        # ── Kicker spinning indicator ────────────────────────────────
        kicker_vel = abs(self.kicker.kicker.get_velocity().value)
        if kicker_vel > 1.0:
            self._kicker_angle += kicker_vel * 0.5  # scale for visual speed
            self.kicker_indicator.setColor(wpilib.Color8Bit(100, 100, 255))  # blue
        else:
            self.kicker_indicator.setColor(wpilib.Color8Bit(60, 60, 60))
        self.kicker_indicator.setAngle(self._kicker_angle % 360)

        # ── Shooter flywheel spinning indicator ──────────────────────
        flywheel_vel = abs(self.shooter.shooter_right.get_velocity().value)
        if flywheel_vel > 1.0:
            self._flywheel_angle += flywheel_vel * 0.3
            self.flywheel_indicator.setColor(wpilib.Color8Bit(255, 60, 60))  # red
        else:
            self.flywheel_indicator.setColor(wpilib.Color8Bit(60, 60, 60))
        self.flywheel_indicator.setAngle(self._flywheel_angle % 360)

        # ── Hood indicator ───────────────────────────────────────────
        hood_vel = abs(self.shooter.shooter_hood.get_velocity().value)
        if hood_vel > 1.0:
            self.hood.setColor(wpilib.Color8Bit(255, 200, 200))  # light red
        else:
            self.hood.setColor(wpilib.Color8Bit(180, 180, 180))  # gray
