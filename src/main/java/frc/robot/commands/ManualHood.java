package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TurretSubsystem;
import java.util.function.DoubleSupplier;

public class ManualHood extends Command {

  TurretSubsystem turret;
  DoubleSupplier input;

  public ManualHood(TurretSubsystem turret, DoubleSupplier input) {
    this.turret = turret;
    this.input = input;
  }

  @Override
  public void execute() {
    if (Math.abs(input.getAsDouble()) > 0.2) {
      turret.hood(input.getAsDouble() * 0.4);
    }
  }

  @Override
  public void end(boolean interrupted) {
    turret.hood(0);
  }
}
