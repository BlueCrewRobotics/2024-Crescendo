package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 *
 */
public class ClimberSubsystem extends SubsystemBase {

    private final TalonFX motor1 = new TalonFX(Constants.ELEVATOR_MOTOR_1_ID);
    private final TalonFX motor2 = new TalonFX(Constants.ELEVATOR_MOTOR_2_ID);

    private final VelocityVoltage shooterVelocity = new VelocityVoltage(1);
    private final SimpleMotorFeedforward shooterFeedForward = new SimpleMotorFeedforward(Constants.shooterKS, Constants.shooterKV, Constants.shooterKA);

    private final DutyCycleOut shooterDutyCycle = new DutyCycleOut(0);

    public ClimberSubsystem() {
        TalonFXConfiguration motorFXConfig = new TalonFXConfiguration();
/*
          .withMotorOutput(new MotorOutputConfigs()
                  .withInverted(InvertedValue.Clockwise_Positive))
          .withCurrentLimits(new CurrentLimitsConfigs()
                  .withSupplyCurrentLimit(15)
                  .withStatorCurrentLimitEnable(true)
                  .withSupplyCurrentThreshold(10));
*/

        motorFXConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        motor1.getConfigurator().apply(motorFXConfig);
        motor2.getConfigurator().apply(motorFXConfig);
    }



}
