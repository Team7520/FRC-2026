package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TurretSubsystem;
import java.util.function.DoubleSupplier;

public class IndexSpin extends Command {

  TurretSubsystem turret;
  DoubleSupplier leftBumper;

  public IndexSpin(TurretSubsystem turret) {
    this.turret = turret;
  }

  @Override
  public void execute() {
    turret.feeder(0.95);
    turret.index(0.95);
  }

  @Override
  public void end(boolean interrupted) {
    turret.feeder(0);
    turret.index(-0.10);
  }
}
