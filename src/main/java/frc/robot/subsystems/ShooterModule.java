package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;

import frc.robot.Constants;

import static frc.robot.Constants.SHOOTER_MAX_ROTATIONS_PER_SECOND;
import static frc.robot.Constants.SHOOTER_METERS_PER_ROTATION;

public class ShooterModule {

    private final TalonFX topShooterMotor = new TalonFX(Constants.SHOOTER_TOP_MOTOR_ID);
    private final TalonFX bottomShooterMotor = new TalonFX(Constants.SHOOTER_BOTTOM_MOTOR_ID);

    private final VelocityVoltage shooterVelocity = new VelocityVoltage(1);
    private final SimpleMotorFeedforward shooterFeedForward = new SimpleMotorFeedforward(Constants.shooterKS, Constants.shooterKV, Constants.shooterKA);

    private final DutyCycleOut shooterDutyCycle = new DutyCycleOut(0);

    public ShooterModule() {

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

        topShooterMotor.getConfigurator().clearStickyFaults();
        topShooterMotor.getConfigurator().apply(leftMotorFXConfig);
        bottomShooterMotor.getConfigurator().clearStickyFaults();
        bottomShooterMotor.getConfigurator().apply(rightMotorFXConfig);
    }

    public void stop() {
        topShooterMotor.stopMotor();
        bottomShooterMotor.stopMotor();
    }

    public void shoot (double speed) {

        shooterVelocity.Velocity = SHOOTER_MAX_ROTATIONS_PER_SECOND * speed;
        shooterVelocity.FeedForward = shooterFeedForward.calculate(shooterVelocity.Velocity * SHOOTER_METERS_PER_ROTATION);

         topShooterMotor.setControl(shooterVelocity);
         bottomShooterMotor.setControl(shooterVelocity);

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