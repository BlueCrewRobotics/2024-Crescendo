package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.lib.math.Conversions;
import frc.robot.Constants;

import static frc.robot.Constants.SHOOTER_MAX_ROTATIONS_PER_SECOND;
import static frc.robot.Constants.SHOOTER_METERS_PER_ROTATION;

public class Shooter extends SubsystemBase{

    private final TalonFX leftShooterMotor = new TalonFX(Constants.SHOOTER_LEFT_MOTOR_ID);
    private final TalonFX rightShooterMotor = new TalonFX(Constants.SHOOTER_RIGHT_MOTOR_ID);

    private final VelocityVoltage shooterVelocity = new VelocityVoltage(1);
    private final SimpleMotorFeedforward shooterFeedForward = new SimpleMotorFeedforward(Constants.shooterKS, Constants.shooterKV, Constants.shooterKA);

    private final DutyCycleOut shooterDutyCycle = new DutyCycleOut(0);

    public Shooter(){

    }

    private void configureMotors() {

        TalonFXConfiguration leftMotorFXConfig = new TalonFXConfiguration()
                .withMotorOutput(new MotorOutputConfigs()
                        .withInverted(InvertedValue.Clockwise_Positive))
                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withSupplyCurrentLimit(15)
                        .withStatorCurrentLimitEnable(true)
                        .withSupplyCurrentThreshold(10));

        TalonFXConfiguration rightMotorFXConfig =  new TalonFXConfiguration()
                .withMotorOutput(new MotorOutputConfigs()
                        .withInverted(InvertedValue.CounterClockwise_Positive))
                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withSupplyCurrentLimit(15)
                        .withStatorCurrentLimitEnable(true)
                        .withSupplyCurrentThreshold(10));

        leftShooterMotor.getConfigurator().clearStickyFaults();
        leftShooterMotor.getConfigurator().apply(leftMotorFXConfig);
        rightShooterMotor.getConfigurator().clearStickyFaults();
        rightShooterMotor.getConfigurator().apply(rightMotorFXConfig);
    }

    public void stop() {
        leftShooterMotor.stopMotor();
        rightShooterMotor.stopMotor();
    }

    public void shoot (double speed) {

        shooterVelocity.Velocity = SHOOTER_MAX_ROTATIONS_PER_SECOND * speed;
        shooterVelocity.FeedForward = shooterFeedForward.calculate(shooterVelocity.Velocity * SHOOTER_METERS_PER_ROTATION);

         leftShooterMotor.setControl(shooterVelocity);
         rightShooterMotor.setControl(shooterVelocity);

        // leftShooterMotor.setControl(shooterDutyCycle.withOutput());
        // rightShooterMotor.setControl(shooterDutyCycle.withOutput());


    }


    public DutyCycleOut getShooterDutyCycle() {
        return shooterDutyCycle;
    }

    public double getShooterSpeed() {
        return shooterVelocity.Velocity / SHOOTER_MAX_ROTATIONS_PER_SECOND;
    }

    public VelocityVoltage getShooterVelocity() {
        return shooterVelocity;
    }

}