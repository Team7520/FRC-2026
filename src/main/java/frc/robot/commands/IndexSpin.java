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
    turret.setFeeder(1);
    turret.setIndexer(-1);
  }

  @Override
  public void end(boolean interrupted) {
    turret.setFeeder(0);
    turret.setIndexer(0);
  }
}
