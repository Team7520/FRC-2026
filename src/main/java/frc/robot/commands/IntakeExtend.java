package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeExtend extends SequentialCommandGroup {
  public IntakeExtend(IntakeSubsystem intake) {
    addCommands(intake.extendIntake());
  }
}
