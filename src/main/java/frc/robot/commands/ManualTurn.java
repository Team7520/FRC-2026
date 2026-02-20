package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import java.util.function.DoubleSupplier;

public class ManualTurn extends Command {
  private final TurretSubsystem turretSubsystem;
  private final DoubleSupplier input;
  private static final double MANUAL_ADJUSTMENT_RATE = -0.3; // Rotations per execution

  public ManualTurn(TurretSubsystem turretSubsystem, DoubleSupplier input) {
    this.turretSubsystem = turretSubsystem;
    this.input = input;
    addRequirements(turretSubsystem);
  }

  @Override
  public void execute() {
    if (Math.abs(input.getAsDouble()) > 0.2) {
      turretSubsystem.turn(input.getAsDouble() * MANUAL_ADJUSTMENT_RATE);
    }
  }

  @Override
  public void end(boolean interrupted) {
    turretSubsystem.turn(0);
  }
}
