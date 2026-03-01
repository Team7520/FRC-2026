package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import java.util.function.DoubleSupplier;

public class IntakeExtend extends Command {

  IntakeSubsystem intake;
  DoubleSupplier leftBumper;

  public IntakeExtend(IntakeSubsystem intake) {
    this.intake = intake;
    addRequirements(intake);
  }

  @Override
  public void execute() {
    intake.extend(1);
  }

  @Override
  public void end(boolean interrupted) {}
}
