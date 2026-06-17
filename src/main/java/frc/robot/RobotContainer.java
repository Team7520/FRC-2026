// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ManualHood;
import frc.robot.commands.ManualTurn;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;

  private final TurretSubsystem turret;
  private final IntakeSubsystem intake;
  private final ClimberSubsystem climber;
  private final LedSubsystem leds;

  // Controller
  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  public final AprilTagSystem aprilTagSystem = new AprilTagSystem();

  private double speedCutoff = 1;
  private double turnCutoff = 0.7;

  //   public Map<Command, String> autoNames = new HashMap<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        // ModuleIOTalonFX is intended for modules with TalonFX drive, TalonFX turn, and
        // a CANcoder
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));

        // The ModuleIOTalonFXS implementation provides an example implementation for
        // TalonFXS controller connected to a CANdi with a PWM encoder. The
        // implementations
        // of ModuleIOTalonFX, ModuleIOTalonFXS, and ModuleIOSpark (from the Spark
        // swerve
        // template) can be freely intermixed to support alternative hardware
        // arrangements.
        // Please see the AdvantageKit template documentation for more information:
        // https://docs.advantagekit.org/getting-started/template-projects/talonfx-swerve-template#custom-module-implementations
        //
        // drive =
        // new Drive(
        // new GyroIOPigeon2(),
        // new ModuleIOTalonFXS(TunerConstants.FrontLeft),
        // new ModuleIOTalonFXS(TunerConstants.FrontRight),
        // new ModuleIOTalonFXS(TunerConstants.BackLeft),
        // new ModuleIOTalonFXS(TunerConstants.BackRight));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        break;
    }
    turret = new TurretSubsystem(drive);
    intake = new IntakeSubsystem();
    climber = new ClimberSubsystem();
    leds = new LedSubsystem();

    NamedCommands.registerCommand("test", Commands.print("WHAHHAHH"));
    registerNamedCommands();

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    // MARK: - AUTOS
    // Set up SysId routines
    // autoChooser.addOption(
    //     "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    // autoChooser.addOption(
    //     "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    // autoChooser.addOption(
    //     "Drive SysId (Quasistatic Forward)",
    //     drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Drive SysId (Quasistatic Reverse)",
    //     drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // autoChooser.addOption(
    //     "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    // autoNames.put(drive.getAutonomousCommand("Basic"), "outpost");

    autoChooser.addOption("pure climb auto", drive.getAutonomousCommand("middle"));
    // autoNames.put(drive.getAutonomousCommand("middle"), "mid auto");

    autoChooser.addOption(
        "middle depot and climb", drive.getAutonomousCommand("middle outpost climb"));

    autoChooser.addOption(
        "depot double swipe + depot", drive.getAutonomousCommand("depot side trench auto"));

    autoChooser.addOption(
        "outpost double swipe + outpost climbless",
        drive.getAutonomousCommand("climbless outpost double swipe"));

    autoChooser.addOption(
        "outpost double swipe + climb", drive.getAutonomousCommand("climb outpost double swipe"));

    autoChooser.addOption(
        "outpost with bump", drive.getAutonomousCommand("bump climb outpost double swipe"));

    autoChooser.addOption(
        "depot bump auto", drive.getAutonomousCommand("bump depot side trench auto"));

    autoChooser.addOption(
        "2.5 outpost", drive.getAutonomousCommand("2.5 bump climb outpost double swipe"));

    autoChooser.addOption(
        "2.5 depot", drive.getAutonomousCommand("2.5 bump depot side trench auto"));

    autoChooser.addOption("no one will know", drive.getAutonomousCommand("h"));

    autoChooser.addOption(
        "closer 2.5 outpost",
        drive.getAutonomousCommand("closer 2.5 bump climb outpost double swipe"));

    autoChooser.addOption(
        "closer 2.5 depot", drive.getAutonomousCommand("closer 2.5 bump depot side trench auto"));

    // Configure the button bindings
    configureButtonBindings();
  }

  // MARK: - NAMED CMDS

  private void registerNamedCommands() {
    NamedCommands.registerCommand("Turret on", turret.shootAuto(true));
    NamedCommands.registerCommand("Turret off", turret.shootAuto(false));
    NamedCommands.registerCommand("feed index", new InstantCommand(() -> turret.feed(0.916)));
    NamedCommands.registerCommand("feed off", new InstantCommand(() -> turret.feed(0)));
    NamedCommands.registerCommand("intake spin", new InstantCommand(() -> intake.runIntake(0.5)));
    NamedCommands.registerCommand("intake off", new InstantCommand(() -> intake.runIntake(0)));
    NamedCommands.registerCommand(
        "Deploy Climber",
        Commands.run(() -> climber.moveToPosition(-55)).until(() -> climber.atTarget(-55)));
    NamedCommands.registerCommand(
        "Climb",
        Commands.run(() -> climber.moveToPosition(-11.5)).until(() -> climber.atTarget(-11.5)));
    NamedCommands.registerCommand("intake out", intake.extendIntake());
    NamedCommands.registerCommand("intake in", intake.retractIntake());
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    /*
        intake wheels, also intake fully out - left trigger
        indexer  - right trigger
        climber up = start
        climber down = back
        intake retract = hold left bumper, let go = intake extend
        wheels x formation = x button
        reverse index = y
        turret on = a, b for off
        reverse intake = right bumper
        reset gyro to 180 = pov up

        all manuals on operator

        if time, point heading to moving direction
    */

    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driver.getLeftY() * 0.6,
            () -> -driver.getLeftX() * 0.6,
            () -> -driver.getRightX() * 0.6));

    /* DEFAULT COMMANDS */

    // turret.setDefaultCommand(turret.aautoAim());

    // MARK: - DRIVER BUTTONS

    // driver
    //     .leftTrigger()
    //     .whileTrue(Commands.run(() -> intake.runIntake(0.6), intake))
    //     .onFalse(Commands.runOnce(intake::stopAll, intake));

    driver
        .rightTrigger()
        .whileTrue(turret.shootWeak())
        .onFalse(new InstantCommand(() -> turret.stopAll()));

    driver
        .leftTrigger()
        .whileTrue(intake.extendIntake().andThen((() -> intake.runIntake(0.6))))
        .onFalse(new InstantCommand(() -> intake.stopAll()));

    driver
        .leftBumper()
        .whileTrue(intake.slowRetract())
        .onFalse(intake.extendIntake())
        .whileFalse(intake.extendIntake());

    // driver.povLeft().onTrue(Commands.run(() -> climber.resetPosition()));

    // Reset gyro to 0° when B button is pressed

    // MARK: - OPERATOR

    // Manual Intake Controls

    // Manual Turret Controls
    new Trigger(() -> Math.abs(operator.getLeftX()) > 0.1)
        .whileTrue(new ManualTurn(turret, () -> operator.getLeftX()));

    new Trigger(() -> Math.abs(operator.getRightY()) > 0.1)
        .whileTrue(new ManualHood(turret, () -> operator.getRightY()));

    // operator.a().onTrue(intake.extendIntake());
    // operator.b().onTrue(intake.retractIntake());

    operator.povDown().onTrue(new InstantCommand(() -> intake.resetPosition(0)));
    operator.povUp().onTrue(new InstantCommand(() -> intake.resetPosition(-16.5)));

    driver.x().onTrue(Commands.runOnce(() -> drive.resetGyro(0))); // disable for competition
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
