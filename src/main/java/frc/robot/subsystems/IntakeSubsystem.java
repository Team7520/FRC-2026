package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

    private final TalonFX intakeMotor;
    private final TalonFX extendMotor;
    private final TalonFX indexerMotor;
    private final DutyCycleOut duty = new DutyCycleOut(0);
    private final PositionDutyCycle pivotPosReq = new PositionDutyCycle(0);
    double extendedPosition;
    double retractedPosition;

    public IntakeSubsystem() {
        intakeMotor = new TalonFX(1);
        extendMotor = new TalonFX(2); // placeholder ids
        indexerMotor = new TalonFX(3);

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 40.0;

        intakeMotor.getConfigurator().apply(config);
        intakeMotor.setNeutralMode(com.ctre.phoenix6.signals.NeutralModeValue.Brake);
        extendMotor.getConfigurator().apply(config);
        extendMotor.setNeutralMode(com.ctre.phoenix6.signals.NeutralModeValue.Brake);
        indexerMotor.getConfigurator().apply(config);
        indexerMotor.setNeutralMode(com.ctre.phoenix6.signals.NeutralModeValue.Brake);
    }

    public void runIntake(double speed) {
        intakeMotor.setControl(duty.withOutput(speed));
    }

    public void extend() {
        extendMotor.setControl(pivotPosReq.withPosition(extendedPosition));
    }

    public void retract() {
        extendMotor.setControl(pivotPosReq.withPosition(retractedPosition));
    }

    public void runIndexer(double speed) {
        indexerMotor.setControl(duty.withOutput(speed));
    }

    public void stopAll() {
        intakeMotor.setControl(duty.withOutput(0));
        extendMotor.setControl(duty.withOutput(0));
        indexerMotor.setControl(duty.withOutput(0));
    }
}
