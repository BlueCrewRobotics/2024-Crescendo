package frc.robot.subsystems.noteplayer;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import frc.robot.Constants;

public class NotePlayerCTREConfigs implements Constants.NotePlayerConstants, Constants.ShooterConstants {

    public TalonFXConfiguration shooterConfig = new TalonFXConfiguration();

    public TalonFXConfiguration intakeConfig = new TalonFXConfiguration();

    public CANcoderConfiguration armCANcoderConfig = new CANcoderConfiguration();

    public NotePlayerCTREConfigs() {

        /* *** Shooter Configs *** */
        // Current Limits
        shooterConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        shooterConfig.CurrentLimits.SupplyCurrentLimit = 15;
        shooterConfig.CurrentLimits.SupplyCurrentThreshold = 20;
        shooterConfig.CurrentLimits.SupplyTimeThreshold = 0.1;

        // Neutral mode
        shooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        // PID
        shooterConfig.Slot0.kP = 2.0;
        shooterConfig.Slot0.kI = 0.001;
        shooterConfig.Slot0.kD = 0.1;

        // Closed-Loop ramps
        shooterConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.75;
        shooterConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.75;

        // Open-Loop ramps
        shooterConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.75;
        shooterConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.75;

        // Inverted
        shooterConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        /* *** Intake Configs *** */
        // Current Limits
        intakeConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        intakeConfig.CurrentLimits.SupplyCurrentLimit = 10;
        intakeConfig.CurrentLimits.SupplyCurrentThreshold = 15;
        intakeConfig.CurrentLimits.SupplyTimeThreshold = 0.1;

        // Neutral Mode
        intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        // PID
        intakeConfig.Slot0.kP = 0.0;
        intakeConfig.Slot0.kI = 0.0;
        intakeConfig.Slot0.kD = 0.0;

        // Closed-Loop ramps
        intakeConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.6;
        intakeConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.6;
    }
}
