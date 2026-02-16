package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
// import frc.robot.subsystems.ElevatorSubsystem;
import java.util.function.DoubleSupplier;

public class ManualIntakeExtend extends Command {
  private final IntakeSubsystem intakeSubsystem;
  private final DoubleSupplier input;
  private static final double MANUAL_ADJUSTMENT_RATE = 0.3;

  public ManualIntakeExtend(IntakeSubsystem intakeSubsystem, DoubleSupplier input) {
    this.intakeSubsystem = intakeSubsystem;
    this.input = input;
    addRequirements(intakeSubsystem);
  }

  @Override
  public void execute() {
    if (Math.abs(input.getAsDouble()) > 0.2) {
      intakeSubsystem.extendSpin(input.getAsDouble() * MANUAL_ADJUSTMENT_RATE);
    }
  }

  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.extendSpin(0);
  }
}
