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
import frc.robot.commands.IndexSpin;
import frc.robot.commands.IndexSpinReverse;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ManualIntakeExtend;
import frc.robot.generated.TunerConstants;
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
    // Preserve compatibility with season paths; CAN ID 20 now belongs only to the intake.
    NamedCommands.registerCommand("Deploy Climber", Commands.none());
    NamedCommands.registerCommand("Climb", Commands.none());
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
        intake retract = hold left bumper, let go = intake extend
        wheels x formation = x button
        reverse index = y
        turret on = a, b for off
        reverse intake = right bumper
        reset gyro to 180 = pov up

        all manuals on operator

        if time, point heading to moving direction
    */

    /* DEFAULT COMMANDS */

    turret.setDefaultCommand(turret.aautoAim(operator::getRightX, operator::getRightY));

    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driver.getLeftY() * speedCutoff,
            () -> -driver.getLeftX() * speedCutoff,
            () -> -driver.getRightX() * turnCutoff));

    intake.setDefaultCommand(intake.backDrive());

    // MARK: - DRIVER BUTTONS

    // driver
    //     .leftTrigger()
    //     .whileTrue(Commands.run(() -> intake.runIntake(0.6), intake))
    //     .onFalse(Commands.runOnce(intake::stopAll, intake));

    driver
        .leftTrigger()
        .whileTrue(intake.extendIntake().andThen((() -> intake.runIntake(0.6))))
        .onFalse(new InstantCommand(() -> intake.stopAll()));

    driver
        .rightTrigger()
        .whileTrue(turret.shootCommand())
        .onTrue(
            new InstantCommand(
                () -> {
                  turnCutoff = turret.turnCutOff();
                  speedCutoff = turret.speedCutoff();
                }))
        .onFalse(
            new InstantCommand(
                () -> {
                  turnCutoff = 0.7;
                  speedCutoff = 1;
                }));

    // driver.rightTrigger().whileTrue(turret.shootCommand());
    driver
        .leftBumper()
        .whileTrue(intake.slowRetract())
        .onFalse(intake.extendIntake())
        .whileFalse(intake.extendIntake());

    driver.y().whileTrue(new IndexSpinReverse(turret, 0.9));

    // driver
    //     .a()
    //     .onTrue(
    //         new InstantCommand(() -> turret.turretWheels(true))
    //             .alongWith(new InstantCommand(() -> turret.setFeeder(0.8))));

    // driver
    //     .b()
    //     .onTrue(
    //         new InstantCommand(() -> turret.turretWheels(false))
    //             .alongWith(new InstantCommand(() -> turret.setFeeder(0))));

    driver.y().whileTrue(new IndexSpin(turret, 0.5));

    driver
        .rightBumper()
        .whileTrue(new IntakeCommand(intake, -0.916))
        .whileTrue(new IndexSpin(turret, 0.5));

    driver.x().onTrue(Commands.runOnce(() -> drive.stopWithX()));

    // Reset gyro to 0° when B button is pressed

    // MARK: - OPERATOR

    // One small shot adjustment per press; offsets persist while the target/distance changes.
    operator.y().onTrue(turret.increaseShotPower());
    operator.a().onTrue(turret.decreaseShotPower());
    operator.x().onTrue(turret.aimLeft());
    operator.b().onTrue(turret.aimRight());

    // Manual intake extension. The right stick is handled per axis by the turret default command.
    new Trigger(() -> Math.abs(operator.getLeftX()) > 0.2)
        .whileTrue(new ManualIntakeExtend(intake, () -> operator.getLeftX()));

    operator
        .rightTrigger()
        .whileTrue(Commands.run(() -> intake.runIntake(0.5), intake))
        .onFalse(Commands.runOnce(intake::stopAll, intake));

    operator
        .leftTrigger()
        .whileTrue(Commands.run(() -> intake.runIntake(-0.916), intake))
        .onFalse(Commands.runOnce(intake::stopAll, intake));

    operator.povDown().onTrue(new InstantCommand(() -> intake.resetPosition(0)));
    operator.povUp().onTrue(new InstantCommand(() -> intake.resetPosition(-16)));
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
