package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import java.util.function.DoubleSupplier;

public class IntakeSpin extends Command {

  IntakeSubsystem intake;
  DoubleSupplier leftBumper;

  public IntakeSpin(IntakeSubsystem intake) {
    this.intake = intake;
  }

  @Override
  public void initialize() {
    System.out.println("RAHHHH");
  }

  @Override
  public void execute() {
    intake.runIntake(0.3);
  }

  @Override
  public void end(boolean interrupted) {
    intake.runIntake(0);
  }
}
