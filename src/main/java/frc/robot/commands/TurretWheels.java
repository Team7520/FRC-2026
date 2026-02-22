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
    turret.setFlywheelVelocity(40);
    
  }

  @Override
  public void end(boolean interrupted) {
    turret.stopAll();
  }
}
