from __future__ import annotations

import math
import typing

import ntcore
import numpy as np
import phoenix6
import phoenix6.unmanaged
import wpilib
import robotpy_apriltag
from pyfrc.physics.core import PhysicsInterface
from wpilib.simulation import DCMotorSim, DriverStationSim

from wpimath.geometry import Pose3d, Rotation3d, Translation3d, Rotation2d
from wpimath.kinematics import SwerveDrive4Kinematics
from wpimath.system.plant import DCMotor, LinearSystemId
from wpimath.units import kilogram_square_meters

from components.drivetrain import SwerveModule
from generated.tuner_constants_swerve import TunerConstants

from photonlibpy.simulation.visionSystemSim import VisionSystemSim
from photonlibpy.simulation.photonCameraSim import PhotonCameraSim
from photonlibpy.simulation.simCameraProperties import SimCameraProperties

if typing.TYPE_CHECKING:
    from robot import MyRobot


class SimpleTalonFXMotorSim:
    def __init__(
        self, motor: phoenix6.hardware.TalonFX, units_per_rev: float, kV: float
    ) -> None:
        self.sim_state = motor.sim_state
        self.sim_state.set_supply_voltage(12.0)
        self.kV = kV  # volt seconds per unit
        self.units_per_rev = units_per_rev

    def update(self, dt: float) -> None:
        voltage = self.sim_state.motor_voltage
        velocity = voltage / self.kV  # units per second
        velocity_rps = velocity * self.units_per_rev
        self.sim_state.set_rotor_velocity(velocity_rps)
        self.sim_state.add_rotor_position(velocity_rps * dt)


class Falcon500MotorSim:
    def __init__(
        self,
        *motors: phoenix6.hardware.TalonFX,
        # Reduction between motor and encoder readings, as output over input.
        # If the mechanism spins slower than the motor, this number should be
        # greater than one.
        gearing: float,
        moi: kilogram_square_meters,
    ):
        self.falcon = DCMotor.falcon500(len(motors))
        self.plant = LinearSystemId.DCMotorSystem(self.falcon, moi, gearing)
        self.gearing = gearing
        self.sim_states = [motor.sim_state for motor in motors]
        for sim_state in self.sim_states:
            sim_state.set_supply_voltage(12.0)
        self.motor_sim = DCMotorSim(self.plant, self.falcon)

    def update(self, dt: float) -> None:
        voltage = self.sim_states[0].motor_voltage
        self.motor_sim.setInputVoltage(voltage)
        self.motor_sim.update(dt)
        motor_rev_per_mechanism_rad = self.gearing / math.tau
        for sim_state in self.sim_states:
            sim_state.set_raw_rotor_position(
                self.motor_sim.getAngularPosition() * motor_rev_per_mechanism_rad
            )
            sim_state.set_rotor_velocity(
                self.motor_sim.getAngularVelocity() * motor_rev_per_mechanism_rad
            )


class PhysicsEngine:
    def __init__(self, physics_controller: PhysicsInterface, robot: MyRobot):
        self.physics_controller = physics_controller
        self.robot = robot
        self.cancoder_test_offset = 0
        self.sim_balls = (
            ntcore.NetworkTableInstance.getDefault()
            .getStructArrayTopic('/components/shooter/simballs', Pose3d)
            .publish()
        )

        self.kinematics: SwerveDrive4Kinematics = robot.drivetrain.kinematics
        self.swerve_modules: tuple[
            SwerveModule, SwerveModule, SwerveModule, SwerveModule
        ] = robot.drivetrain.modules

        # Calculate drive_motor_rev_to_meters: how many meters the robot travels per motor revolution
        # drive_ratio = motor_rotations / wheel_rotations, so:
        # meters_per_motor_rev = wheel_circumference / drive_ratio
        wheel_circumference = TunerConstants._wheel_radius * math.tau
        drive_motor_rev_to_meters = wheel_circumference / TunerConstants._drive_gear_ratio

        # Motors
        self.wheels = [
            SimpleTalonFXMotorSim(
                module.drive,
                units_per_rev=1 / drive_motor_rev_to_meters,
                kV=6.0,
            )
            for module in robot.drivetrain.modules
        ]
        self.steer = [
            Falcon500MotorSim(
                module.steer,
                gearing=1 / TunerConstants._steer_gear_ratio,
                # measured from MKCad CAD
                # moi=0.0009972,
                moi=0.0009972 * 1,
            )
            for module in robot.drivetrain.modules
        ]

        # Intake roller and sushi: open-loop duty-cycle motors, no gearing
        self.intake_roller_sim = SimpleTalonFXMotorSim(
            self.robot.intake.roller,
            units_per_rev=1.0,
            kV=1.0,
        )
        self.intake_sushi_sim = SimpleTalonFXMotorSim(
            self.robot.intake.sushi,
            units_per_rev=1.0,
            kV=1.0,
        )

        self.manip_motors: list[Falcon500MotorSim] = [
            Falcon500MotorSim(
                self.robot.intake.extend,
                gearing=1 / 49.846,
                moi=0.005,
            ),
            Falcon500MotorSim(
                self.robot.intake.roller,
                gearing=1,
                moi=0.0005,
            ),
            Falcon500MotorSim(
                self.robot.intake.sushi,
                gearing=1,
                moi=0.0005,
            ),
            Falcon500MotorSim(
                self.robot.kicker.kicker,
                gearing=1,
                moi=0.0009972 * 2,
            ),
            Falcon500MotorSim(
                self.robot.shooter.shooter_left,
                gearing=1,
                # 2x 3" Colson wheels (~0.1 kg each, r=0.0381m) + 10%
                moi=0.00016,
            ),
            Falcon500MotorSim(
                self.robot.shooter.shooter_right,
                gearing=1,
                moi=0.00016,
            ),
            Falcon500MotorSim(
                self.robot.shooter.shooter_hood,
                gearing=1,
                moi=0.00016,
            ),
        ]

        # Lagged steer angles for sim — makes turning feel more like the real robot
        self._sim_steer_angles = [0.0] * 4

        self.current_yaw = 0.0
        self.gyro = robot.gyro.pigeon.sim_state  # Access the Pigeon 2's sim state
        self.gyro.set_supply_voltage(12.0)  # Set the supply voltage for simulation

        self.apriltag_layout = robotpy_apriltag.AprilTagFieldLayout.loadField(
            robotpy_apriltag.AprilTagField.k2026RebuiltWelded
        )

        self.vision_sim = VisionSystemSim("ardu_cam-1")
        self.vision_sim.addAprilTags(self.apriltag_layout)

        # Luma P1: OV9281 global shutter, 1280x800, 80h/56v FOV, ~89.6 diagonal
        luma_p1_fov_diag = Rotation2d.fromDegrees(89.6)

        # Realistic vision noise — measured from MIBKN match logs:
        #   Rear camera (best view):  ~5 cm position std dev
        #   Side cameras (oblique):   ~20-25 cm position std dev
        #   Heading:                  ~5-18 deg std dev
        # These pixel-noise values produce distance-dependent pose error
        # that approximates the real camera behavior.
        CALIB_AVG_ERROR_PX = 0.75   # average tag-corner error (pixels)
        CALIB_STD_DEV_PX = 0.35     # per-frame variation in corner error
        CAM_AVG_LATENCY_S = 0.035   # 35 ms average pipeline latency
        CAM_LATENCY_STD_S = 0.010   # 10 ms jitter

        def _make_luma_p1_properties() -> SimCameraProperties:
            props = SimCameraProperties.OV9281_1280_720()
            props.setCalibrationFromFOV(1280, 800, luma_p1_fov_diag)
            props.setCalibError(CALIB_AVG_ERROR_PX, CALIB_STD_DEV_PX)
            props.setAvgLatency(CAM_AVG_LATENCY_S)
            props.setLatencyStdDev(CAM_LATENCY_STD_S)
            return props

        self.camera_bl = PhotonCameraSim(robot.vision.camera_rl, _make_luma_p1_properties())
        self.camera_bl.setMaxSightRange(4.0)

        self.camera_br = PhotonCameraSim(robot.vision.camera_rr, _make_luma_p1_properties())
        self.camera_br.setMaxSightRange(4.0)

        self.camera_back = PhotonCameraSim(robot.vision.camera_back, _make_luma_p1_properties())
        self.camera_back.setMaxSightRange(4.0)

        self.camera_front = PhotonCameraSim(robot.vision.camera_front, _make_luma_p1_properties())
        self.camera_front.setMaxSightRange(4.0)

        self.vision_sim.addCamera(
            self.camera_bl,
            self.robot.vision.camera_rl_offset,
        )
        self.vision_sim.addCamera(
            self.camera_br,
            self.robot.vision.camera_rr_offset,
        )
        self.vision_sim.addCamera(
            self.camera_back,
            self.robot.vision.camera_back_offset,
        )
        self.vision_sim.addCamera(
            self.camera_front,
            self.robot.vision.camera_front_offset,
        )

        # Sim-only chooser: which alliance won autonomous?
        # The FMS game message is the alliance whose HUB goes inactive FIRST —
        # the LOSER of auto — so the labels here are the inverse of the raw char.
        self._auton_winner_chooser: wpilib.SendableChooser = wpilib.SendableChooser()
        self._auton_winner_chooser.setDefaultOption("Unknown (both active)", "")
        self._auton_winner_chooser.addOption("Red Won Auto  → Blue inactive first", "B")
        self._auton_winner_chooser.addOption("Blue Won Auto → Red inactive first",  "R")
        wpilib.SmartDashboard.putData("Sim/AutonWinner", self._auton_winner_chooser)

    def update_sim(self, now: float, tm_diff: float) -> None:
        # Enable the Phoenix6 simulated devices
        # TODO: delete when phoenix6 integrates with wpilib
        if wpilib.DriverStation.isEnabled():
            phoenix6.unmanaged.feed_enable(0.1)

        # Inject the simulated FMS game message so hub_shoot_indicator() works in sim
        DriverStationSim.setGameSpecificMessage(
            self._auton_winner_chooser.getSelected() or ""
        )

        if False:
            poses: list[Pose3d] = []
            for b in self.robot.turret.balls:
                pose = Pose3d(
                    Translation3d(b.xpos, b.ypos, b.zpos),
                    Rotation3d(0, 0, 0),
                )
                b.xpos += b.xvel * tm_diff
                b.ypos += b.yvel * tm_diff
                b.zpos += b.zvel * tm_diff
                if b.zpos >= 0.075:
                    # Calculate new Z velocity with simple gravity
                    b.zvel -= 9.81 * tm_diff
                else:
                    # Ball has hit the ground; stop simulating it
                    b.xvel = 0.0
                    b.yvel = 0.0
                    b.zvel = 0.0

                poses.append(pose)
            if len(poses) > 0:
                self.sim_balls.set(poses)


        for wheel in self.wheels:
            wheel.update(tm_diff)
        for steer in self.steer:
            steer.update(tm_diff)
        for m in self.manip_motors:
            m.update(tm_diff)
        self.intake_roller_sim.update(tm_diff)
        self.intake_sushi_sim.update(tm_diff)

        # Track steer angles with a lag filter (25% per step) so turning in sim
        # feels more like the real robot instead of instantaneous.
        STEER_SIM_ALPHA = 0.35
        for i, module in enumerate(self.robot.drivetrain.modules):
            desired_ang = module.state.angle.radians()
            curr = self._sim_steer_angles[i]
            # Use atan2 to correctly interpolate across the ±π wrap boundary
            error = math.atan2(math.sin(desired_ang - curr), math.cos(desired_ang - curr))
            self._sim_steer_angles[i] = curr + STEER_SIM_ALPHA * error
            sigma = (math.radians(0.5) / math.tau) / 2
            raw = self._sim_steer_angles[i] / math.tau + np.random.normal(loc=0, scale=sigma)
            module.encoder.sim_state.set_raw_position(
                raw - module.mag_offset
            )

        speeds = self.kinematics.toChassisSpeeds((
            self.swerve_modules[0].get(),
            self.swerve_modules[1].get(),
            self.swerve_modules[2].get(),
            self.swerve_modules[3].get(),
        ))

        # Scale sim turning speed to match real-robot behavior
        # (tuned via tools/sim_tuner.py against MIBKN match logs)
        speeds.omega *= 0.38

        self.current_yaw += math.degrees(speeds.omega * tm_diff)
        sigma = (math.radians(0.5) / math.tau) / 2
        # yaw_jitter = np.random.normal(loc=0, scale=sigma)
        yaw_jitter = 0
        self.gyro.set_raw_yaw(self.current_yaw + yaw_jitter)

        self.physics_controller.drive(speeds, tm_diff)
        self.vision_sim.update(self.robot.drivetrain.get_pose())

