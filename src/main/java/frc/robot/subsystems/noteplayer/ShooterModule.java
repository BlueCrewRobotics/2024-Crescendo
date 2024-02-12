package frc.robot.subsystems.noteplayer;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;

import frc.robot.Constants;

public class ShooterModule implements Constants.ShooterConstants {

    private final TalonFX topShooterMotor = new TalonFX(SHOOTER_TOP_MOTOR_ID);
    private final TalonFX bottomShooterMotor = new TalonFX(SHOOTER_BOTTOM_MOTOR_ID);


    private VelocityVoltage shooterVelocity = new VelocityVoltage(1);
    private SimpleMotorFeedforward shooterFeedForward = new SimpleMotorFeedforward(shooterKS, shooterKV, shooterKA);

    private final DutyCycleOut shooterDutyCycle = new DutyCycleOut(0);

    public ShooterModule() {
/*
        TalonFXConfiguration leftMotorFXConfig = new TalonFXConfiguration()
                .withMotorOutput(new MotorOutputConfigs()
                        .withInverted(InvertedValue.Clockwise_Positive))
                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withSupplyCurrentLimit(15)
                        .withStatorCurrentLimitEnable(true)
                        .withSupplyCurrentThreshold(10));

        TalonFXConfiguration rightMotorFXConfig = new TalonFXConfiguration()
                .withMotorOutput(new MotorOutputConfigs()
                        .withInverted(InvertedValue.CounterClockwise_Positive))
                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withSupplyCurrentLimit(15)
                        .withStatorCurrentLimitEnable(true)
                        .withSupplyCurrentThreshold(10));
*/
        TalonFXConfiguration motorFXConfig = new TalonFXConfiguration();

        /*
        motorFXConfig.MotionMagic.MotionMagicCruiseVelocity = 20; // Target cruise velocity
        motorFXConfig.MotionMagic.MotionMagicAcceleration = 40; // Target acceleration of  (0.5 seconds)
        motorFXConfig.MotionMagic.MotionMagicJerk = 200; // Target jerk (0.1 seconds)
*/

        motorFXConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = .75;
        motorFXConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.75;

        topShooterMotor.getConfigurator().clearStickyFaults();
        topShooterMotor.getConfigurator().apply(motorFXConfig);
        bottomShooterMotor.getConfigurator().clearStickyFaults();
        bottomShooterMotor.getConfigurator().apply(motorFXConfig);
    }

    public void stop() {
        topShooterMotor.stopMotor();
        bottomShooterMotor.stopMotor();
    }

    public void shoot(double speed) {

//        topShooterMotor.setControl(new DutyCycleOut(speed));
//        bottomShooterMotor.setControl(new DutyCycleOut(speed));

        shooterVelocity = new VelocityVoltage(SHOOTER_MAX_ROTATIONS_PER_SECOND * speed);
        shooterFeedForward = new SimpleMotorFeedforward(shooterKS, shooterKV, shooterKA);

//        shooterVelocity.Velocity = SHOOTER_MAX_ROTATIONS_PER_SECOND * speed;
        shooterVelocity.FeedForward = shooterFeedForward.calculate(shooterVelocity.Velocity * SHOOTER_METERS_PER_ROTATION);
        //shooterVelocity.withAcceleration(25);

        System.out.println(("Shooter velocity set to: " + shooterVelocity.Velocity));
        topShooterMotor.setControl(shooterVelocity);
        bottomShooterMotor.setControl(shooterVelocity);

        // leftShooterMotor.setControl(shooterDutyCycle.withOutput());
        // rightShooterMotor.setControl(shooterDutyCycle.withOutput());


    }

    public double getShooterTopVelocity() {
        return topShooterMotor.getVelocity().getValue().doubleValue();
    }

    public double getShooterBottomVelocity() {
        return topShooterMotor.getVelocity().getValue().doubleValue();
    }

    public long getShooterTopEncoderPos() {
        return (long) (topShooterMotor.getPosition().getValue() * 2048.0d);
    }

    public long getShooterBottomEncoderPos() {
        return (long) (topShooterMotor.getPosition().getValue() * 2048.0d);
    }


}