package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TurretSubsystem;

public class TurretPivot extends Command {
  TurretSubsystem turret;

  public TurretPivot(TurretSubsystem turret) {
    this.turret = turret;
  }

  @Override
  public void execute() {
    turret.moveToPosition(-26);
  }
}
