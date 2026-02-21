package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import java.util.function.DoubleSupplier;

public class IntakeReverse extends Command {

  IntakeSubsystem intake;
  DoubleSupplier leftBumper;

  public IntakeReverse(IntakeSubsystem intake) {
    this.intake = intake;
  }

  @Override
  public void execute() {
    intake.runIntake(-0.6);
  }

  @Override
  public void end(boolean interrupted) {
    intake.runIntake(0);
  }
}
