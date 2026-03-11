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
    // turret.setFeeder(1);
    turret.setIndexer(speed);
  }

  @Override
  public void end(boolean interrupted) {
    // turret.setFeeder(0);
    turret.setIndexer(0);
  }
}
