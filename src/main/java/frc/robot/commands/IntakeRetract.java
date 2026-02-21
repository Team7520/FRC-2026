package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import java.util.function.DoubleSupplier;

public class IntakeRetract extends Command {

  IntakeSubsystem intake;
  DoubleSupplier leftBumper;

  public IntakeRetract(IntakeSubsystem intake) {
    this.intake = intake;
    addRequirements(intake);
  }

  @Override
  public void execute() {
    intake.retract();
  }

  @Override
  public void end(boolean interrupted) {}
}
