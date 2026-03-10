package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TurretSubsystem;

public class IndexSpin extends Command {

  TurretSubsystem turret;
  double speed;

  public IndexSpin(TurretSubsystem turret, double speed) {
    this.turret = turret;
    this.speed = speed;
  }

  @Override
  public void execute() {
    // Only index when the turret is safe to shoot (not wrapping)
    if (turret.canShoot()) {
      turret.setIndexer(speed);
    } else {
      turret.setIndexer(0);
    }
  }

  @Override
  public void end(boolean interrupted) {
    // turret.setFeeder(0);
    turret.setIndexer(0);
  }
}
