from magicbot import StateMachine, state, tunable

from components.shooter import ShooterComponent
from components.kicker import KickerComponent
from components.singulator import SingulatorComponent


class GasPump(StateMachine):
    """Staggered shooting sequence to avoid voltage sag.

    Spins up shooter first, then kicker, then singulator — each waiting
    for the previous motor to stabilise before starting the next.
    """

    shooter: ShooterComponent
    kicker: KickerComponent
    singulator: SingulatorComponent

    shooter_rps = tunable(30.0)
    kicker_current_threshold = tunable(20.0)

    def shoot(self) -> None:
        """Begin the shooting sequence (if not already running)."""
        if self.current_state == self.waiting.name:
            self.next_state_now(self.shooter_spin_up)

    def stop(self) -> None:
        """Abort shooting and kill all motors."""
        self.next_state_now(self.waiting)

    # ------------------------------------------------------------------
    # States
    # ------------------------------------------------------------------

    @state(first=True, must_finish=True)
    def waiting(self) -> None:
        self.shooter.stop()
        self.kicker.kicker_off()
        self.singulator.singulator_off()

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
