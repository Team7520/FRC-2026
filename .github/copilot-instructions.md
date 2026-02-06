# Copilot Instructions

## Big Picture
- WPILib command-based project layered with AdvantageKit logging; `Robot.java` drives the mode switch between REAL/SIM/REPLAY via `Constants.currentMode` and always starts the logger.
- `RobotContainer.java` owns subsystem wiring, chooses IO implementations per mode (real hardware uses `ModuleIOTalonFX`, sim uses `ModuleIOSim`, replay is no-IO), registers PathPlanner autos, and binds the Xbox controller.

## Drive System
- `subsystems/drive/Drive.java` is the core swerve subsystem: it locks around odometry updates, streams Phoenix signals through `PhoenixOdometryThread`, feeds a `SwerveDrivePoseEstimator`, and configures PathPlanner `AutoBuilder` plus the custom `LocalADStarAK` pathfinder.
- Module hardware is abstracted behind `ModuleIO`; real TalonFX+CANcoder logic lives in `ModuleIOTalonFX`, which queues timestamped samples. New hardware variants should mirror this interface so `Module` can continue to log and optimize setpoints.
- Gyro data follows the same IO pattern (`GyroIO` and `GyroIOPigeon2`), delivering buffered yaw readings for high-rate odometry; keep updates within the `Drive.odometryLock` guard when extending.

## Commands & Controls
- Teleop behaviors live in `commands/DriveCommands.java`; joystick helpers already square inputs and flip field-relative control for the red alliance. Reuse these helpers when writing new drive commands.
- Characterization routines (feedforward, wheel radius, SysId) are already exposed in `RobotContainer` and rely on AdvantageKit logging—follow their pattern when adding diagnostics so data ends up under the same log keys.

## PathPlanner Integration
- Autonomous routines load from `src/main/deploy/pathplanner/**`; update those assets in tandem with code. Autos are surfaced through `LoggedDashboardChooser`, so add new options there for visibility.
- Dynamic obstacle avoidance routes through `util/LocalADStarAK.java`; call its `setStartPosition`, `setGoalPosition`, and `setDynamicObstacles` before expecting `isNewPathAvailable()` to flip true.

## AdvantageKit & Logging
- All IO inputs are annotated with `@AutoLog`; refresh via `Logger.processInputs` and publish derived values with `Logger.recordOutput` (see `Drive.java` keys like `SwerveStates/*` and `Odometry/Robot`). Maintain this naming scheme for consistency in AdvantageScope.
- Replay mode runs without hardware IO; when writing replay-specific code, guard hardware-only calls behind `Constants.currentMode` checks.

## Replay Safety
- Non-deterministic sources break log replay: avoid raw FPGA timestamps (`Timer.getFPGATimestamp()`), random numbers, unordered iteration, NetworkTables inputs, and hardware APIs called outside an IO layer. Use `Timer.getTimestamp()` and ensure new data flows through AdvantageKit inputs so replay matches real runs.
- Keep robot logic single-threaded; only spin extra threads inside IO implementations (e.g., sensor polling) and never call `Logger.recordOutput`/`Logger.processInputs` from those threads. Sync thread data through the IO input structs so the main loop remains deterministic.
- Do not touch Driver Station state or timestamps before `Logger.start()`. Instantiate subsystems after the logger comes online, and expect IO input structs to hold default values until the first `periodic()` call updates them.

## Build & Simulation Workflow
- Desktop sim uses `PowerShell> .\gradlew simulateExternalJavaRelease` with the WPILib JDK path already passed by VS Code; toggle between SIM and REPLAY by editing `Constants.simMode`.
- Deploy to the roboRIO with `PowerShell> .\gradlew deploy`; the `eventDeploy` task auto-commits on branches starting with `event`, so expect local commits after deploys if you follow that branching scheme.
- Run AdvantageKit log replay through `PowerShell> .\gradlew replayWatch`; the build disables the HAL sim GUI by default for compatibility.
- `spotlessApply` runs before `compileJava`; invoke `PowerShell> .\gradlew spotlessApply` manually if Gradle reports formatting issues.

## Generated & Vendor Artifacts
- `frc/robot/generated/TunerConstants.java` and `frc/robot/BuildConstants.java` are regenerated (CTRE Tuner X and Gradle gversion respectively); update source tools instead of editing these files.
- Vendor libraries are tracked in `vendordeps/*.json` (CTRE Phoenix 6, REVLib, AdvantageKit, PathPlanner); keep versions aligned with the generated code when upgrading hardware configs.

## Hardware Notes
- Competition robot runs SDS MK5n (R3 ratio) swerve: Kraken X60 drive motors, Kraken X44 steer motors, and CTRE CANcoders on every module; all CAN IDs and module geometry live in `frc/robot/generated/TunerConstants.java` and must remain generated-only.
- CAN bus infrastructure relies on a single CANivore named `CANivore`; never rename the bus or reuse those IDs when adding devices.
- IMU is a CTRE Pigeon 2.0 on the same CAN-FD bus; the team holds a 2026 Phoenix Pro license, so prefer Phoenix Pro control modes/features when extending IO implementations.
- RIO is a roboRIO 2.0; it is acceptable to lean on RIO 2-specific optimizations and expect deployment solely to that controller.
- The RIO CAN 2.0 bus currently only hosts the REV PDH, but leave room for future non-CTRE devices on CAN 2.0 and additional vendor motor controllers.
- Use YASS YAMS libraries; Limelight 4 and PhotonVision vision pipelines are planned, so leave extension points for camera IO.

## Extending the System
- Follow the IO interface pattern (interface + AutoLog inputs + real/sim implementations) for new subsystems so they integrate with AdvantageKit logging and replay out of the box.
- Any new Phoenix 6 signals should register with `PhoenixOdometryThread` (and a timestamp queue) just like `ModuleIOTalonFX` to preserve synchronized sampling across devices.