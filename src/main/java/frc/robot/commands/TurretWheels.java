package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TurretSubsystem;
import java.util.function.DoubleSupplier;

public class TurretWheels extends Command {

  TurretSubsystem turret;
  DoubleSupplier leftBumper;

  public TurretWheels(TurretSubsystem turret) {
    this.turret = turret;
  }

  @Override
  public void execute() {
    turret.spinWheels(0.3);
    turret.feeder(0.2);
    turret.index(0.2);
  }

  @Override
  public void end(boolean interrupted) {
    turret.spinWheels(0);
    turret.feeder(0);
    turret.index(0);
  }
}
