package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends Command {

  IntakeSubsystem intake;
  double speed;

  public IntakeCommand(IntakeSubsystem intake, double speed) {
    this.intake = intake;
    this.speed = speed;
  }

  // @Override
  // public void initialize() {

  // }

  @Override
  public void execute() {
    intake.extendIntake();
    intake.runIntake(speed);
  }

  @Override
  public void end(boolean interrupted) {
    intake.stopAll();
  }
}
