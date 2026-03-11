package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TurretSubsystem;

public class IndexSpinReverse extends Command {

  TurretSubsystem turret;
  double speed;

  public IndexSpinReverse(TurretSubsystem turret, double speed) {
    this.turret = turret;
    this.speed = speed;
  }

  @Override
  public void execute() {
    turret.setFeeder(-0.8);
    turret.setIndexer(speed);
  }

  @Override
  public void end(boolean interrupted) {
    // turret.setFeeder(0);
    turret.setIndexer(0);
  }
}
