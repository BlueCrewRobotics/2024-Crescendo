package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.ctre.phoenix6.hardware.CANcoder;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/**
 * This is the Swerve Drive Subsystem, based on team 364's code
 */
public class SwerveSubsystem extends SubsystemBase {
    private SwerveDrivePoseEstimator swervePoseEstimator;
    public SwerveModule[] mSwerveMods;
    private AHRS gyro;

    private final VisionPoseEstimator vision;

    private Field2d field = new Field2d();

    private PIDController rotationPIDController;

    public SwerveSubsystem() {
        gyro = new AHRS(SPI.Port.kMXP);
        gyro.reset();

        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        resetModulesToAbsolute();

        vision = new VisionPoseEstimator();
        
        var stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.1); // TODO: Tune the standard deviations
        var visionStdDevs = VecBuilder.fill(1, 1, 1);
        swervePoseEstimator =
                new SwerveDrivePoseEstimator(
                        Constants.Swerve.swerveKinematics,
                        getGyroYaw(),
                        getModulePositions(),
                        new Pose2d(),
                        stateStdDevs,
                        visionStdDevs);

        rotationPIDController = new PIDController(5, 0, 0);
        rotationPIDController.enableContinuousInput(0, 360);
        rotationPIDController.setTolerance(1);

        // Configure PathPlanner Auto Builder
        AutoBuilder.configureHolonomic(
                this::getPose,
                this::setPose,
                this::getRobotRelativeSpeeds,
                this::driveRobotRelative,
                Constants.PathPlannerConstants.pathFollowerConfig,
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this
        );

        PathPlannerLogging.setLogActivePathCallback((poses) -> field.getObject("path").setPoses(poses));

        // Logging callback for the active path, this is sent as a list of poses
        PathPlannerLogging.setLogActivePathCallback((poses) -> {
            // Do whatever you want with the poses here
            field.getObject("path").setPoses(poses);
        });

        SmartDashboard.putData("Field", field);
    }

    /**
     * Drives the {@link SwerveSubsystem} Subsystem
     * @param translation A {@link Translation2d} representing the desired X and Y speed in meters per second
     * @param rotation The desired angular velocity in radians per second
     * @param fieldRelative Controls whether it should be driven in field or robot relative mode
     * @param isOpenLoop Controls whether it drives in open loop
     */
    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    getHeading()
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }

    /**
     * Sets the states of each module
     * @param desiredStates An array of the {@link SwerveModuleState} for each {@link SwerveModule}
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    /**
     * Turns all the wheels to face inwards creating and "X" shape to lock the robot in place if the robot is not currently moving
     */
    public void xLockWheels() {
        if(getModuleStates()[0].speedMetersPerSecond < 0.1) {
            SwerveModuleState[] desiredStates = {
                    new SwerveModuleState(0.0, Rotation2d.fromDegrees(45)),
                    new SwerveModuleState(0.0, Rotation2d.fromDegrees(135)),
                    new SwerveModuleState(0.0, Rotation2d.fromDegrees(135)),
                    new SwerveModuleState(0.0, Rotation2d.fromDegrees(45))
            };
            for(SwerveModule mod : mSwerveMods){
                mod.setAngle(desiredStates[mod.moduleNumber]);
            }
        }
    }

    /**
     * @return An array containing the {@link SwerveModuleState} of each {@link SwerveModule}
     */
    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    /**
     * @return An array of the {@link SwerveModulePosition} of each {@link SwerveModule}
     */
    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    /**
     * @return The {@link Pose2d} of the robot according to the {@link SwerveDrivePoseEstimator}
     */
    public Pose2d getPose() {
        return swervePoseEstimator.getEstimatedPosition();
    }

    /**
     * @param pose The {@link Pose2d} to set the {@link SwerveDrivePoseEstimator} to
     */
    public void setPose(Pose2d pose) {
        swervePoseEstimator.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }

    /**
     * @return The speeds of each {@link SwerveModule} as {@link ChassisSpeeds} Required by Path Planner
     */
    public ChassisSpeeds getRobotRelativeSpeeds() {
        return Constants.Swerve.swerveKinematics.toChassisSpeeds(getModuleStates());
    }

    /**
     * Required by Path Planner, drives the robot given {@link ChassisSpeeds}
     * @param speeds the ChassisSpeeds to drive the robot with
     */
    public void driveRobotRelative(ChassisSpeeds speeds) {
        SwerveModuleState[] states = Constants.Swerve.swerveKinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.Swerve.maxSpeed);
        setModuleStates(states);
    }

    /**
     * @return The heading reported by the {@link SwerveDrivePoseEstimator} as a {@link Rotation2d}
     */
    public Rotation2d getHeading(){
        return getPose().getRotation();
    }

    /**
     * Sets the {@link SwerveDrivePoseEstimator} heading
     * @param heading The heading of the robot
     */
    public void setHeading(Rotation2d heading){
        swervePoseEstimator.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), heading));
    }

    /**
     * Resets the {@link SwerveDrivePoseEstimator} heading
     */
    public void zeroHeading(){
        swervePoseEstimator.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), new Rotation2d()));
    }

    /**
     * @return The Robot yaw as reported by the NavX
     */
    public Rotation2d getGyroYaw() {
        return Constants.Swerve.invertGyro ? (Rotation2d.fromDegrees(360-gyro.getYaw())) : Rotation2d.fromDegrees(gyro.getYaw());
    }

    /**
     * @return The angular velocity of the robot as reported by the NavX
     */
    public float getGyroYawSpeed() {
        return Constants.Swerve.invertGyro ? -gyro.getRawGyroZ() : gyro.getRawGyroZ();
    }

    /**
     * Resets each {@link SwerveModule} based on the {@link CANcoder} and angle offset
     */
    public void resetModulesToAbsolute(){
        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }

    /**
     * Used for driving the robot during teleop
     * @param translationSup {@link DoubleSupplier} for the forwards/backwards speed as a percentage
     * @param strafeSup {@link DoubleSupplier} for the left/right speed as a percentage
     * @param rotationSup {@link DoubleSupplier} for the angular velocity as a percentage
     * @param robotCentricSup {@link BooleanSupplier} for driving in robot or field centric mode
     */
    public void teleopDriveSwerve(DoubleSupplier translationSup, DoubleSupplier strafeSup,
                                  DoubleSupplier rotationSup, BooleanSupplier robotCentricSup) {
        /* Get Values, Deadband*/
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);
        double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);

        drive(new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed),
                rotationVal * Constants.Swerve.maxAngularVelocity,
                !robotCentricSup.getAsBoolean(),
                true);
    }

    /**
     * Creates an angular velocity based on a target angle, and the direction the robot is currently facing
     * @param targetAngle The angle the robot should turn to
     * @return The angular velocity as a percentage
     */
    public double rotationPercentageFromTargetAngle(Rotation2d targetAngle) {
        rotationPIDController.reset();
        return MathUtil.clamp(rotationPIDController.calculate(getGyroYaw().getDegrees()%360, targetAngle.getDegrees()%360), -0.25, 0.25);
    }

    /**
     * Get the angle to a given position
     * @param coords The coordinates to return the angle to
     * @return The angle from your current position to the given coordinates
     */
    public Rotation2d getAngleToPose(Translation2d coords) {
        return Rotation2d.fromRadians(Math.atan(
                (coords.getX() - getPose().getX()) / (coords.getY()-getPose().getY())
        ));
    }

    /**
     * Cancels the command currently running on this subsystem unless it's the default command
     */
    public void cancelCurrentCommand() {
        if(this.getCurrentCommand() != this.getDefaultCommand()) {
            CommandScheduler.getInstance().cancel(this.getCurrentCommand());
        }
    }

    // **** Commands ****
    /**
     * Used for driving the robot during teleop while rotating to the angle reported by the D-Pad of the controller
     * @param translationSup {@link DoubleSupplier} for the forwards/backwards speed as a percentage
     * @param strafeSup {@link DoubleSupplier} for the left/right speed as a percentage
     * @param targetDegrees The target angle
     * @param robotCentricSup {@link BooleanSupplier} for driving in robot or field centric mode
     */
    public Command teleopDriveSwerveAndRotateToAngleCommand(DoubleSupplier translationSup, DoubleSupplier strafeSup,
                                                            double targetDegrees, BooleanSupplier robotCentricSup) {

        return this.run(
                () -> teleopDriveSwerve(
                        translationSup,
                        strafeSup,
                        () -> rotationPercentageFromTargetAngle(Rotation2d.fromDegrees(targetDegrees)),
                        robotCentricSup
                )
        );
    }

    @Override
    public void periodic(){
        swervePoseEstimator.update(getGyroYaw(), getModulePositions());

        // Correct pose estimate with vision measurements
        var visionEst = vision.getEstimatedGlobalPose();
        visionEst.ifPresent(
                est -> {
                    var estPose = est.estimatedPose.toPose2d();
                    // Change our trust in the measurement based on the tags we can see
                    var estStdDevs = vision.getEstimationStdDevs(estPose);

                    swervePoseEstimator.addVisionMeasurement(
                            est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
                });

        SmartDashboard.putNumber("Swerve Estimator X", swervePoseEstimator.getEstimatedPosition().getX());
        SmartDashboard.putNumber("Swerve Estimator Y", swervePoseEstimator.getEstimatedPosition().getY());
        if(visionEst.isPresent()) {
            SmartDashboard.putNumber("Vision Estimator X", visionEst.get().estimatedPose.getX());
            SmartDashboard.putNumber("Vision Estimator Y", visionEst.get().estimatedPose.getY());
        }
        for(SwerveModule mod : mSwerveMods) {
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " CANcoder", mod.getCANcoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
        }
    }
}