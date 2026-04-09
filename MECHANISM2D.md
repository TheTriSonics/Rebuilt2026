# Viewing the Robot Mechanism2d in Simulation

The robot publishes a side-profile Mechanism2d visualization called **"Robot Side View"** that shows the intake arm, rollers, kicker, shooter flywheel, and hood in real time.

## Quick Start

1. Launch the simulator:

   ```bash
   python -m robotpy sim
   ```

2. Open **AdvantageScope** and connect to the simulator:
   - File > Connect to Simulator (or enter `localhost` / `127.0.0.1` as the address)
   - It should auto-connect to NetworkTables on port 5810

3. Add the Mechanism2d widget:
   - Click the **+** tab at the top to add a new tab
   - Choose the **Mechanism** tab type
   - In the left sidebar, drag `SmartDashboard/Robot Side View` into the main area

4. Enable the robot in the sim GUI (set mode to Teleop and click Enable) to see the mechanisms move.

## What You'll See

| Element | Color | What it shows |
|---------|-------|---------------|
| Orange arm | Intake arm pivot angle from the CANcoder |
| Green dot at arm tip | Intake roller active (spins when running) |
| Cyan dot at arm tip | Sushi roller active (spins when running) |
| Blue indicator | Kicker wheel speed |
| Red indicator | Shooter flywheel speed |
| Gray flap | Hood (turns pink when active) |
| Yellow arrow | Ball exit direction (static) |
| Gray rectangle | Robot body outline (static) |

## Troubleshooting

- **Nothing shows up in AdvantageScope**: Make sure you're on the Mechanism tab type, not a line graph tab. The widget only renders in the dedicated Mechanism view.
- **Widget is blank**: The Mechanism2d only publishes after the robot code initializes. Wait for the sim GUI to fully load and show "Robot Side View" in the NetworkTables tree.
- **Arm doesn't move**: The intake motor sim needs to be enabled. Switch to Teleop mode in the sim GUI and press buttons that trigger intake movement (or set `target_position` via the dashboard).
- **Flywheels don't spin**: Trigger the shooting sequence via the operator controller or set `shooter.active = True` on the dashboard.
