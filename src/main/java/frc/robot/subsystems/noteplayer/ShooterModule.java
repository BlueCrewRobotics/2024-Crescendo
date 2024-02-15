package frc.robot.subsystems.noteplayer;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;

import frc.robot.Constants;

public class ShooterModule implements Constants.ShooterConstants {

    private final TalonFX topShooterMotor = new TalonFX(SHOOTER_TOP_MOTOR_ID);
    private final TalonFX bottomShooterMotor = new TalonFX(SHOOTER_BOTTOM_MOTOR_ID);

    private final NotePlayerCTREConfigs ctreConfigs = new NotePlayerCTREConfigs();

    private VelocityVoltage shooterVelocity = new VelocityVoltage(1);
    private SimpleMotorFeedforward shooterFeedForward = new SimpleMotorFeedforward(shooterKS, shooterKV, shooterKA);

    private final DutyCycleOut shooterDutyCycle = new DutyCycleOut(0);

    public ShooterModule() {
        topShooterMotor.getConfigurator().clearStickyFaults();
        topShooterMotor.getConfigurator().apply(ctreConfigs.shooterConfig);
        bottomShooterMotor.getConfigurator().clearStickyFaults();
        bottomShooterMotor.getConfigurator().apply(ctreConfigs.shooterConfig);
    }

    public void stop() {
        topShooterMotor.stopMotor();
        bottomShooterMotor.stopMotor();
    }

    public void spinPercentage(double speed) {
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

    public void spinMetersPerSecond(double mps) {
        shooterVelocity.Velocity = mps/SHOOTER_METERS_PER_ROTATION;
        shooterVelocity.FeedForward = shooterFeedForward.calculate(mps);

        System.out.println("Shooter Velocity MPS set to: " + mps);
        topShooterMotor.setControl(shooterVelocity);
        bottomShooterMotor.setControl(shooterVelocity);
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

    public double getShooterTopVelocityMPS() {
        return topShooterMotor.getVelocity().getValue() * SHOOTER_METERS_PER_ROTATION;
    }

    public double getShooterBottomVelocityMPS() {
        return bottomShooterMotor.getVelocity().getValue() * SHOOTER_METERS_PER_ROTATION;
    }

    /**
     *
     * @return Whether both the shooter motors are within {@value SHOOTER_SPEED_ERROR_TOLERANCE}% of the set velocity
     */
    public boolean targetVelocityReached() {
        double epsilon = SHOOTER_SPEED_ERROR_TOLERANCE/100;
        return getShooterTopVelocity() > (shooterVelocity.Velocity * (1-epsilon))
                && getShooterTopVelocity() < (shooterVelocity.Velocity * (1+epsilon))
                && getShooterBottomVelocity() > (shooterVelocity.Velocity * (1-epsilon))
                && getShooterBottomVelocity() < (shooterVelocity.Velocity * (1+epsilon));
    }
}