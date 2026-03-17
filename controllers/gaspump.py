from magicbot import StateMachine, state, tunable

from components.shooter import ShooterComponent
from components.kicker import KickerComponent


class GasPump(StateMachine):
    """Staggered shooting sequence to avoid voltage sag.

    Spins up shooter first, then kicker, then singulator — each waiting
    for the previous motor to stabilise before starting the next.
    """

    shooter: ShooterComponent
    kicker: KickerComponent

    def __init__(self) -> None:
        ...

    def go_shoot(self) -> None:
        self.next_state_now(self.shooter_spin_up)

    def go_shoot_off(self) -> None:
        self.next_state_now(self.shooter_off)

    def go_stop(self) -> None:
        """Abort shooting and kill all motors."""
        self.next_state_now(self.waiting)

    def go_intake(self) -> None:
        self.next_state_now(self.intake_running)

    def go_singulate(self) -> None:
        self.next_state_now(self.singulate)

    def go_eject(self) -> None:
        self.next_state_now(self.eject)

    def go_intake_off(self) -> None:
        self.next_state_now(self.waiting)

    # ------------------------------------------------------------------
    # States
    # ------------------------------------------------------------------

    @state(first=True, must_finish=True)
    def waiting(self, initial_call: bool) -> None:
        if initial_call:
            # self.intake.off()
            ...

    @state(must_finish=True)
    def shooter_off(self, initial_call: bool) -> None:
        if initial_call:
            self.shooter.stop()

    @state(must_finish=True)
    def shooter_spin_up(self) -> None:
        self.shooter.spin_up()
        if self.shooter.is_at_speed():
            self.next_state(self.kicker_spin_up)

    @state(must_finish=True)
    def kicker_spin_up(self) -> None:
        self.shooter.spin_up()
