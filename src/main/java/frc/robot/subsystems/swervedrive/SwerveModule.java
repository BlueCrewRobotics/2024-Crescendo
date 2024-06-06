package frc.robot.subsystems.swervedrive;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.math.Conversions;
import frc.lib.util.SwerveModuleConstants;
import frc.robot.Constants;
import frc.robot.Robot;

/**
 * Represents a Swerve Module on the Swerve Drivetrain
 */
public class SwerveModule implements Constants.Swerve {
    public int moduleNumber;
    private Rotation2d angleOffset;

    private TalonFX mAngleMotor;
    private TalonFX mDriveMotor;
    private CANcoder angleEncoder;

    private SwerveCTREConfigs ctreConfigs = new SwerveCTREConfigs();

    private final SimpleMotorFeedforward driveFeedForward = new SimpleMotorFeedforward(driveKS, driveKV, driveKA);

    /* drive motor control requests */
    private final DutyCycleOut driveDutyCycle = new DutyCycleOut(0);
    private final VelocityVoltage driveVelocity = new VelocityVoltage(0);

    /* angle motor control requests */
    private final PositionVoltage anglePosition = new PositionVoltage(0);

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants){
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;
        
        /* Angle Encoder Config */
        angleEncoder = new CANcoder(moduleConstants.cancoderID);
        angleEncoder.getConfigurator().apply(ctreConfigs.swerveCANcoderConfig);

        /* Angle Motor Config */
        mAngleMotor = new TalonFX(moduleConstants.angleMotorID);
        mAngleMotor.getConfigurator().apply(ctreConfigs.swerveAngleFXConfig);
        resetToAbsolute();

        /* Drive Motor Config */
        mDriveMotor = new TalonFX(moduleConstants.driveMotorID);
        mDriveMotor.getConfigurator().apply(ctreConfigs.swerveDriveFXConfig);
        mDriveMotor.getConfigurator().setPosition(0.0);

        mDriveMotor.setSafetyEnabled(true);
        mAngleMotor.setSafetyEnabled(true);
    }

    /**
     * @param desiredState The desired {@link SwerveModuleState}
     * @param isOpenLoop Controls driving in open or closed loop
     */
    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
        desiredState = SwerveModuleState.optimize(desiredState, getState().angle); 
        mAngleMotor.setControl(anglePosition.withPosition(desiredState.angle.getRotations()));
        setSpeed(desiredState, isOpenLoop);
    }

    /**
     * Sets the speed of the drive motor
     * @param desiredState The desired {@link SwerveModuleState}
     * @param isOpenLoop Controls driving in open or closed loop
     */
    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop){
        // Scale speed by cosine of angle error. This scales down movement perpendicular to the desired
        // direction of travel that can occur when modules change directions. This results in smoother driving.
        desiredState.speedMetersPerSecond *= desiredState.angle.minus(Rotation2d.fromRotations(mAngleMotor.getPosition().getValue())).getCos();
        if(isOpenLoop){
            driveDutyCycle.Output = desiredState.speedMetersPerSecond / maxSpeed;
            mDriveMotor.setControl(driveDutyCycle);
        }
        else {
            driveVelocity.Velocity = Conversions.MPSToRPS(desiredState.speedMetersPerSecond, wheelCircumference);
            driveVelocity.FeedForward = driveFeedForward.calculate(desiredState.speedMetersPerSecond);
            mDriveMotor.setControl(driveVelocity);
        }
    }

    /**
     * Sets the angle of the wheel
     * @param desiredState The desired {@link SwerveModuleState}
     */
    public void setAngle(SwerveModuleState desiredState){
        mAngleMotor.setControl(anglePosition.withPosition(desiredState.angle.getRotations()));
    }

    /**
     * @return The angle of the {@link CANcoder}
     */
    public Rotation2d getCANcoder(){
        return Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValue());
    }

    /**
     * Resets the {@link SwerveModule} based on the {@link CANcoder} and angle offset
     */
    public void resetToAbsolute(){
        double absolutePosition = getCANcoder().getRotations() - angleOffset.getRotations();
        mAngleMotor.setPosition(absolutePosition);
    }

    /**
     * @return The {@link SwerveModuleState} of the {@link SwerveModule}
     */
    public SwerveModuleState getState(){
        return new SwerveModuleState(
            Conversions.RPSToMPS(mDriveMotor.getVelocity().getValue(), wheelCircumference),
            Rotation2d.fromRotations(mAngleMotor.getPosition().getValue())
        );
    }

    /**
     * @return The {@link SwerveModulePosition} of the {@link SwerveModule}
     */
    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            Conversions.rotationsToMeters(mDriveMotor.getPosition().getValue(), wheelCircumference),
            Rotation2d.fromRotations(mAngleMotor.getPosition().getValue())
        );
    }

    public void setDriveVoltage(double voltage) {
        mDriveMotor.setVoltage(voltage);
    }

    public double getAngleError() {
        return mAngleMotor.getClosedLoopError().getValue();
    }

    public double getDriveVoltage() {
        return mDriveMotor.getMotorVoltage().getValue();
    }

    public double getDriveSpeedError() {
        return mDriveMotor.getClosedLoopError().getValue();
    }
}