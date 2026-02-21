package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import java.util.function.DoubleSupplier;

public class IntakeCommand extends Command {

  IntakeSubsystem intake;
  DoubleSupplier leftBumper;

  public IntakeCommand(IntakeSubsystem intake) {
    this.intake = intake;
    addRequirements(intake);
  }

  @Override
  public void execute() {
    intake.extend();
  }

  @Override
  public boolean isFinished() {
    return intake.atTarget(-16.4794921875);
  }
}
