from magicbot import StateMachine, state, tunable

from components.shooter import ShooterComponent
from components.kicker import KickerComponent
from components.singulator import SingulatorComponent
from components.intake import IntakeComponent


class GasPump(StateMachine):
    """Staggered shooting sequence to avoid voltage sag.

    Spins up shooter first, then kicker, then singulator — each waiting
    for the previous motor to stabilise before starting the next.
    """

    shooter: ShooterComponent
    kicker: KickerComponent
    singulator: SingulatorComponent
    intake: IntakeComponent

    shooter_rps = tunable(50.0)
    kicker_current_threshold = tunable(20.0)

    def go_shoot(self) -> None:
        self.next_state_now(self.shooter_spin_up)

    def go_stop(self) -> None:
        """Abort shooting and kill all motors."""
        self.next_state_now(self.waiting)

    def go_intake(self) -> None:
        self.next_state_now(self.intake_running)

    def go_singulate(self) -> None:
        self.next_state_now(self.singulate)

    def go_eject(self) -> None:
        self.next_state_now(self.eject)

    # ------------------------------------------------------------------
    # States
    # ------------------------------------------------------------------

    @state(first=True, must_finish=True)
    def waiting(self) -> None:
        self.shooter.stop()
        self.kicker.kicker_off()
        self.singulator.singulator_off()
        self.intake.intake_off()

    @state(must_finish=True)
    def shooter_spin_up(self) -> None:
        self.shooter.spin_up(self.shooter_rps)
        self.kicker.kicker_off()
        self.singulator.singulator_off()
        if self.shooter.is_at_speed():
            self.next_state(self.kicker_spin_up)

    @state(must_finish=True)
    def kicker_spin_up(self) -> None:
        self.shooter.spin_up(self.shooter_rps)
        self.kicker.kicker_forward()
        self.singulator.singulator_off()
        stator_current = abs(self.kicker.kicker.get_stator_current().value)
        if stator_current < self.kicker_current_threshold:
            self.next_state(self.singulator_spin_up)

    @state(must_finish=True)
    def singulator_spin_up(self) -> None:
        self.shooter.spin_up(self.shooter_rps)
        self.kicker.kicker_forward()
        self.singulator.singulator_forward()

    @state(must_finish=True)
    def intake_running(self) -> None:
        self.shooter.stop()
        self.kicker.kicker_reverse()
        self.singulator.singulator_forward()
        #self.intake.rotate_down()
        self.intake.intake_on()

    @state(must_finish=True)
    def singulate(self) -> None:
        self.shooter.stop()
        self.kicker.kicker_reverse()
        self.singulator.singulator_forward()
        #self.intake.rotate_up()
        self.intake.intake_off()

    @state(must_finish=True)
    def eject(self) -> None:
        self.shooter.stop()
        self.kicker.kicker_reverse()
        self.singulator.singulator_reverse()
        #self.intake.rotate_down()
        self.intake.intake_reverse()
        
