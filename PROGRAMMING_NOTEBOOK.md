# Programming Notebook — FRC 4003 TriSonics, Rebuilt 2026

## Team Profile

**Team Number:** 4003
**Team Name:** TriSonics
**Season:** 2026 Rebuilt
**Language:** Python (RobotPy + MagicBot)
**Repository:** TheTriSonics/Rebuilt2026

### Contributors

| Name | Role | Highlights |
|------|------|------------|
| Justin Buist | Programming Mentor | Architecture, vision, drivetrain, and keeping things from catching fire |
| Student Coder(s) | Student Programmers | Subsystems, tuning, driver practice fixes, and plenty of "it works now" commits |
| IAmABigTurtle | Student Programmer | Early sim work, vision integration, CANCoder configs |
| dowdjr | Student Programmer | Autonomous modes, shooter curves, operator controls |
| Ronan061 | Student Programmer | PRs, code review merges, shooter tuning |
| Cole & Jon | Students | "cole and jon made an intake" — and so they did |

---

## Season Goals

1. Build a competitive swerve drive robot that can shoot fuel cells accurately
2. Develop a multi-camera vision system for reliable field localization
3. Get students writing and deploying real robot code, not just watching
4. Have working autonomous modes before our first competition

---

## Phase 1: SimBalls — Where It All Began

**Branch:** `simballs`
**Dates:** January 3 – March 14, 2026

The project started in the simulator. Before we had any hardware to break, we wanted a codebase that could drive, shoot, and follow paths — all on a laptop screen. The idea was that we could develop the hard stuff (swerve kinematics, shooting math, path following) without needing the robot in front of us.

### Project Skeleton (Jan 3–17)

> `78df9e4` — "Basic project with a pinned 2026 library version"
> `a21a385` — "Basics for a 2026 project"
> `3954a11` — "Initial project skeleton with sim"

We started with the bare minimum: a RobotPy project pinned to the 2026 WPILib beta, a physics.py for simulation, and the CTRE-generated swerve constants. IAmABigTurtle got the sim constants matched to last year's bot so we could drive around immediately.

### Shooting Simulation (Jan 21–22)

> `a7c662b` — "Experiment with simulating shot balls"
> `7f2e35f` — "Shooting on the fly seems kinda working"
> `08fcfb9` — "Decent enough example for shooting on the fly for now."

One of the more ambitious early experiments: actually simulating the flight of shot balls in the sim GUI, including chassis velocities. This let us visualize what "shoot on the fly" would look like before we had a real shooter. It was rough but it gave us confidence that the math was heading in the right direction.

### First Contact with Hardware (Jan 24–28)

> `57623d4` — "Swapping in constants for swerve chassis"
> `bfccd9b` — "Fixing ids in our code to match physical bot"
> `c2a5433` — "Moved to a fused CANCoder config"
> `1d8c404` — "Fixed Driving"

The swerve chassis showed up and suddenly all our sim constants were wrong. CAN IDs didn't match, encoder offsets were off, units were confused. This is the "it works in sim but not on the real robot" phase that every FRC team knows well. We swapped in the real CTRE Wizard values and got wheels turning.

> `15a90c5` — "Tweaking drivetrain with chassis bot"

This one represents a lot of floor time. The commit message is short but the session was long.

### Subsystem Buildout (Jan 31 – Feb 21)

Students started building components in earnest:

> `a84b58e` — "cole and jon made an intake"
> `fcc7ef1` — "Adding in climber system"
> `fd8e7ee` — "made a kicker (#3)"
> `32121a6` — "Add PID to climber motor (#2)"

These were some of our first pull requests. Students were learning git, writing components, and submitting PRs for review. The commit messages tell the story — they're straightforward and honest.

The Tanker controller (our drive mode state machine) got pulled out of robot.py:

> `2d65507` — "Moved 'drive to point' out of robot.py and into its own controller (#1)"

And then things started accelerating. The shooter, singulator, turret, LEDs, and battery monitor all came online in a burst during a February lab session:

> `7d5d9c7` — "Add shooter component with leader/follower motors and status LED"
> `2359666` — "Add turret motor and CANCoder with position control"
> `2383bb7` — "Add in battery monitor so we quit destroying them in practice"

That battery monitor commit is worth noting. We were burning through batteries because nobody was watching voltage levels during practice. The monitor was a direct response to that — a lesson learned the expensive way.

### The Singulator Saga (Feb 21)

The singulator (the mechanism that feeds fuel cells one at a time into the shooter) was a multi-commit adventure in a single day:

> `d836938` — "added a not working singulater button"
> `cdf7463` — "Getting singulator working"
> `f4d880e` — "Singulator works in duty cycle"
> `27696d0` — "Switch singulator from VelocityVoltage to DutyCycleOut"
> `7d179d3` — "Switch singulator to velocity PID control with internal encoder"
> `19501ee` — "Limits set so the singulator spins well under load"
> `398fa97` — "Less power. Aaron was nervous about that"

That last one is a classic. Sometimes the right engineering decision is "the person standing next to the robot looks concerned, so maybe dial it back."

### Vision — The Long Road (Feb 11 – Mar 14)

Vision was probably the single biggest effort of the season. It started simple:

> `aca3a69` — "Basic vision"
> `23d4193` — "Adding photonvision"
> `5b10c32` — "PhotonLib deploys to robot now"

Then came the work of making it actually useful:

> `577cee7` — "Fix camera positions and make a visual to use in AdvantageScope"
> `696ed8f` — "Add 254-inspired heading lock, vision rejection, fusion, and gyro fallback"
> `79788cb` — "Enable 2nd camera and fix offsets"
> `b0146ee` — "speeding up vision"
> `f7181f2` — "Optimize vision execute() to reduce per-cycle overhead"
> `144e024` — "Trim up vision's hot paths as much as possible"

The 254-inspired work was a turning point. We studied how top teams handle vision data — rejecting bad measurements, fusing multiple cameras with inverse-variance weighting, and falling back to gyro-only odometry when vision is unreliable. This gave us a much more stable pose estimate.

Vision was also where we learned that `print()` statements in hot loops are a bad idea:

> `bdf22b6` — "Vision set to right camera names, removed print() from singulator"

Performance matters when your code runs every 20 milliseconds.

### The 254 Ideas Branch and Its Reversal (Feb 25–27)

> `696ed8f` — "Add 254-inspired heading lock, vision rejection, fusion, and gyro fallback"
> `5d7ed43` — "Merge pull request #5 from TheTriSonics/simballs-254ideas"
> `35d1094` — "Revert 'Merge pull request #5 from TheTriSonics/simballs-254ideas'"

We tried pulling in a batch of vision and heading improvements inspired by Team 254's approach. It got merged, then reverted, then the good parts were brought back individually. This is normal — sometimes a big change needs to be broken into smaller pieces to figure out what's actually helping.

---

## Phase 2: AlphaBot — The First Real Robot

**Branch:** `alphabot` (branched from simballs on March 14)
**Dates:** March 14, 2026

AlphaBot was our first delivery from the build team. It was more mechanically complex than what we'd end up competing with — notably, it had a **turret**. A turret that could rotate, aim at the hub, and track targets while the drivetrain moved independently.

The turret was both a blessing and a curse. It forced us to solve harder aiming math (calculating shot angles from the turret position, not the robot center) but it also meant more things could go wrong.

### Turret Work (Mar 3–12)

> `6a513fd` — "Latest updates -- working on turret pointing"
> `c200c4e` — "Fix turret angle work"
> `b51ee7c` — "Turret angles all worked out."
> `eba59a1` — "Moving the turret aiming targets to a dictionary list."
> `829d923` — "Updated to add operator controls for the hub, left, and right turret positions."
> `1fbb425` — "calculate the shot angle from the turret position, not robot center."

That last commit represents a key insight: when you have a turret, your shot angle calculation needs to account for where the turret is on the robot, not just where the robot is on the field. Sounds obvious in hindsight, but it took some head-scratching to get right.

### Dynamic Shooting (Mar 6–8)

> `d7b6765` — "Dynamic shooter speed calcs"
> `6169fb3` — "Shot distance data worksheet"
> `80d48e5` — "Shoot on the fly work"

We built a spreadsheet of shot distances vs. required shooter speeds and fit a curve to the data. This became the shooting curve formula that Rob would later inherit and refine.

### Driver Practice and Pragmatism (Mar 11)

> `ec71450` — "Change intake current limit to the rotate motor. Lock turret for the night for driver practice."

Sometimes you need to lock the turret at 180 degrees and let the drivers practice. The turret was still being tuned but the drive team needed seat time. Engineering is about trade-offs, and this was one of them.

### Autonomous Beginnings (Mar 14)

> `fb64784` — "Auton basics worked out - also setting position and heading when disabled works"
> `b8995bd` — "tried to add a second autonomous"
> `74779fd` — "timing issue"

The first autonomous modes came together on AlphaBot. Setting the robot's initial pose during the disabled period (using vision) was a key feature — it means the robot knows where it is before autonomous even starts.

### What AlphaBot Taught Us

AlphaBot was our proving ground. The turret aiming math, the vision fusion pipeline, the shot calculator, the GasPump shooting sequence — all of these were developed or stabilized on AlphaBot. When Rob came along, we didn't have to figure this stuff out from scratch. We just had to adapt it.

---

## Phase 3: Rob — The Competition Bot

**Branch:** `rob` (branched from alphabot on March 14)
**Dates:** March 15 – present

When Rob arrived, there was genuine excitement. This was the competition robot. The whole team had been waiting for it. Rob was mechanically simpler than AlphaBot — no turret, but a fixed "double" shooter that could feed more than one fuel cell at a time. Less complexity meant fewer things to break at competition.

We branched the code and started adapting.

### The Transition (Mar 15–16)

> `6382874` — "Pre Beta bot"
> `ea5b182` — "Basics kinda working"

"Basics kinda working" is the most honest commit message in the repo. The drivetrain drove, the intake moved, the shooter spun. Not perfect, not competition-ready, but alive.

### Intake and Shooter Tuning for Rob (Mar 17)

> `b012d11` — "Intake setpoints and config ready for ROB"
> `580d08b` — "Fiddling with code to make debugging shooter easier"
> `e066f5a` — "Stop drivetrain snapping after shooting"

Rob's fixed shooter meant we didn't need turret code anymore, but we did need to re-tune everything for the new geometry. The drivetrain was "snapping" — jerking to a new heading after completing a shot — and that needed fixing for driver confidence.

### Autonomous Modes Come Together (Mar 17–19)

> `9bc237d` — "Add RightRed autonomous mode with event marker support"
> `3fbc390` — "First auton works. yay."
> `bbe31ff` — "Adding 2nd path to an auton"
> `3a10f95` — "Another center auton worked out."

"First auton works. yay." might be our favorite commit message of the season. Autonomous modes use Choreo trajectory files with event markers that trigger intaking and shooting at specific points along the path. Getting the first one working end-to-end was a milestone.

### Three Cameras Online (Mar 18)

> `54a3150` — "Enable 3rd camera"
> `c370858` — "Oops -- mistake in adding 3rd camera"
> `84bc07a` — "Add floor to vision std devs to prevent pose drift when stationary"

Rob runs three PhotonVision cameras (Rear_Right, Rear_Left, Rear). The third camera initially caused some issues — turns out copy-paste errors in camera configs are easy to make. We also discovered that when the robot is sitting still, vision measurements with very low standard deviations can cause the pose estimate to drift. Adding a floor to the standard deviations fixed this.

### Shooting Refinement (Mar 18–19)

> `63a4ca9` — "Add vision verification for disabled, add cubic curve for rps calc."
> `7c59e7e` — "Changed shooting curve formula, added phase timer and indicator"
> `32dd582` — "Don't allow lob shots unless neither bumper has been pressed; keeps us from targeting a lob spot when releasing bumpers after targetting the hub"

The shooting curve evolved from AlphaBot's version to a cubic polynomial fit tuned for Rob's specific shooter geometry. The lob shot logic is a nice detail — it prevents the robot from accidentally targeting a lob position when the driver releases the bumper buttons after aiming at the hub.

### Alliance-Aware Features (Mar 18)

> `a89b015` — "Added Left/Right flipper and Dashboard setup functions"

The left/right flipper lets us mirror autonomous modes and targeting positions for red vs. blue alliance. You set it on the dashboard before the match and the code handles the coordinate transforms.

---

## Technical Deep Dives

### Vision Pipeline Evolution

Our vision system went through at least three major iterations:

1. **Single camera, raw pose** (Feb 11) — Just get PhotonVision reading tags. Good enough to prove the concept.
2. **Multi-camera with basic fusion** (Feb 25–27) — Two cameras, 254-inspired rejection criteria, inverse-variance fusion. Reject measurements with high ambiguity, too far from expected pose, or unreasonable Z values.
3. **Three-camera production pipeline** (Mar 18) — All three cameras feeding into a Kalman filter with dynamic standard deviations based on tag count and distance. Floor on std devs to prevent stationary drift.

**Rejection criteria we settled on:**
- Ambiguity >= 0.2 (the measurement is too uncertain)
- Z coordinate > 0.2m (the robot isn't flying, so something is wrong)
- Tag distance > 2.0m (accuracy drops fast with distance)
- Pose divergence > 1.0m from current estimate (outlier rejection)

### The GasPump Shooting Sequence

Named by the students (our controllers have a fuel theme — Tanker for driving, GasPump for shooting). GasPump is a state machine that orchestrates the shooting sequence:

1. Spin up the shooter flywheels
2. Wait for them to reach target RPM
3. Stagger the kicker to avoid voltage sag (we learned this one from brownouts)
4. Feed fuel cells into the shooter

The staggered spin-up was added after we noticed the battery voltage dipping hard when everything spun up at once:

> `5aa2c25` — "Add GasPump staggered shooting sequence controller"

### Swerve Drivetrain Lessons

- **Fused CANCoders** (`c2a5433`) — Using the TalonFX's built-in fusion of the CANCoder signal with the motor's internal encoder gives smoother and more accurate wheel angle feedback.
- **Field-centric driving was backwards** (`c679a53` — "Fix 180 degree field-centric driving and odometry heading error") — Classic bug. Field-centric drive makes the robot go the direction you push the stick regardless of which way the robot is facing. Unless your heading is 180 degrees off, in which case it does the opposite.
- **Heading lock** (`696ed8f`) — When the driver is translating without rotating, we hold the current heading with a PID controller. Keeps the robot from drifting in yaw. Inspired by how Team 254 handles this.

### AI-Assisted Development

We used AI coding tools where they could help:

> `7b657ed` — Claude: "Add Choreo event marker support during path following"
> `8ae8dfe` — Claude: "Fix intake rotation and add debug feedback telemetry"
> `54ec53a` — Ronan061: "Use absolute encoder setpoints for intake rotation" (from Codex branch)
> `ab52fe7` — google-labs-jules: "Fix turret manual target modes and fix vision pose lag"

These tools were useful for well-defined tasks — adding event marker support, fixing specific bugs, wiring up encoder configs. The students reviewed and merged the PRs, which is the important part. The AI writes the code but the team has to understand and own it.

---

## Lessons Learned

### Things That Went Well

- **Simulator-first development** paid off. We had working swerve code, path following, and shooting math before the first chassis arrived.
- **MagicBot's component model** kept the code organized. Each subsystem is self-contained and testable.
- **Starting vision early** gave us time to iterate through multiple approaches. Vision is never easy and the extra weeks mattered.
- **Branching strategy** worked. SimBalls -> AlphaBot -> Rob let us evolve the code without breaking what already worked.

### Things That Were Hard

- **Vision performance** was a constant battle. Every unnecessary computation in the 20ms loop shows up as lag. We spent a lot of time profiling and trimming.
- **Turret math** on AlphaBot was genuinely difficult. Calculating shot angles from a rotating turret that's offset from the robot center involves more trigonometry than anyone signed up for.
- **Battery management** — we had to add a battery monitor component because we kept over-discharging batteries during practice. Not glamorous, but important.
- **The singulator** took an entire day to get working through six different motor control approaches. Sometimes you just have to try things.

### Things We'd Do Differently

- Start with the battery monitor from day one. Don't wait until you've damaged a few.
- Set up AdvantageScope visualization earlier. Being able to see what the vision system thinks it's seeing saves hours of debugging.
- More consistent use of `@feedback` decorators for telemetry. When something goes wrong on the field, dashboard data is all you have.

---

## Timeline Summary

| Date | Milestone |
|------|-----------|
| Jan 3 | Project created, RobotPy 2026 pinned |
| Jan 17 | Simulation skeleton running |
| Jan 22 | Simulated ball shooting working |
| Jan 24 | First hardware (swerve chassis) arrives |
| Jan 28 | Chassis driving under control |
| Jan 31 | First student-built subsystems (intake, climber) |
| Feb 4 | First pull requests from students |
| Feb 11 | PhotonVision integration begins |
| Feb 21 | Shooter, turret, singulator, battery monitor, LEDs all online |
| Feb 25 | 254-inspired vision fusion and heading lock |
| Mar 6 | Turret angles figured out, shot distance calibration |
| Mar 9 | Kicker and shooter tuned |
| Mar 11 | Driver practice with locked turret |
| Mar 14 | AlphaBot: first autonomous modes working, shot calculator extracted |
| Mar 15 | Rob (competition bot) arrives — code branched |
| Mar 16 | "Basics kinda working" |
| Mar 17 | First autonomous modes on Rob |
| Mar 18 | Three cameras online, shooting curve refined, alliance-aware features |
| Mar 19 | Multiple autonomous modes working, competition prep |

---

*This notebook was assembled from the git commit history of the Rebuilt2026 repository. Commit messages are quoted directly where they tell the story best.*
