package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSTalonFXSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

/**
 * Contains all the robot constants
 */
public final class Constants {

    /**
     * All the driver controls constants
     */
    public static final class DriverControls {
        public static final double stickDeadband = 0.1;
        public static final double swerveSensitivityExponent = 1.5; // Should to be >= 1
        public static final double swerveSpeedMultiplier = 0.7; // Should be <= 1
        public static final double swerveRotationMultiplier = 0.7; // Should be <= 1
    }
    /**
     * All the Swerve Subsystem constants
     */
    public static final class Swerve {

        public static final boolean invertGyro = true; // Always ensure Gyro is CCW+ CW-

        public static final COTSTalonFXSwerveConstants chosenModule =
                COTSTalonFXSwerveConstants.SDS.MK4i.Falcon500(COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios.L2);

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(20.5);
        public static final double wheelBase = Units.inchesToMeters(20.5);
        public static final double wheelCircumference = chosenModule.wheelCircumference;

        /* Swerve Kinematics
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
                new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final InvertedValue angleMotorInvert = chosenModule.angleMotorInvert;
        public static final InvertedValue driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final SensorDirectionValue cancoderInvert = chosenModule.cancoderInvert;

        /* Swerve Current Limiting */
        public static final int angleCurrentLimit = 25;
        public static final int angleCurrentThreshold = 40;
        public static final double angleCurrentThresholdTime = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveCurrentLimit = 35;
        public static final int driveCurrentThreshold = 60;
        public static final double driveCurrentThresholdTime = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = 70;//chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.1;
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;

        /* Drive Motor Characterization Values From SYSID */
        public static final double driveKS = 0.08403;
        public static final double driveKV = 2.2341;
        public static final double driveKA = 0.27385;

        /* Swerve Profiling Values */
        /**
         * Meters per Second
         */
        public static final double maxSpeed = 4.9;
        /**
         * Radians per Second
         */
        public static final double maxAngularVelocity = 12.1725; //TODO: This is theoretical!

        /* Neutral Modes */
        public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Brake;
        public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

        /* Module Specific Constants */

        /**
         * Constants for the Front Left Module - Module 0
         */
        public static final class Mod0 {
            public static final int driveMotorID = 0;
            public static final int angleMotorID = 4;
            public static final int canCoderID = 0;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(138.58);
            public static final SwerveModuleConstants constants =
                    new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /**
         * Constants for the Front Right Module - Module 1
         */
        public static final class Mod1 {
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 5;
            public static final int canCoderID = 1;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(204.84);
            public static final SwerveModuleConstants constants =
                    new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /**
         * Constants for the Back Left Module - Module 2
         */
        public static final class Mod2 {
            public static final int driveMotorID = 2;
            public static final int angleMotorID = 6;
            public static final int canCoderID = 2;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(115.35);
            public static final SwerveModuleConstants constants =
                    new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /**
         * Constants for the Back Right Module - Module 3
         */
        public static final class Mod3 {
            public static final int driveMotorID = 3;
            public static final int angleMotorID = 7;
            public static final int canCoderID = 3;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(213.43);
            public static final SwerveModuleConstants constants =
                    new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
    }

    public static final class PhotonVision {

        public static final String cameraName = "testCamera";
        // Cam mounted facing forward, half a meter forward of center, half a meter up from center.
        public static final Transform3d robotToCam =
                new Transform3d(new Translation3d(0.0, 0.0, 0.0), new Rotation3d(Math.toRadians(32), 0, 0));

        // The layout of the AprilTags on the field
        public static final AprilTagFieldLayout tagLayout =
                AprilTagFields.kDefaultField.loadAprilTagLayoutField();

        // The standard deviations of our vision estimated poses, which affect correction rate
        // (Fake values. Experiment and determine estimation noise on an actual robot.)
        public static final Matrix<N3, N1> singleTagStdDevs = VecBuilder.fill(4, 4, 8);
        public static final Matrix<N3, N1> multiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
    }

    public static final class ShooterConstants {

        public enum ShooterStatus {
            PRIMED,
            UNREADY
        }

        public enum ShooterMode {
            PICKUP,
            SPEAKER,
            AMP,
            TRAP
        }
    }

    /**
     * Contains all the Constants used by Path Planner
     */
    public static final class PathPlannerConstants {

        public static final PathConstraints pathConstraints =
                new PathConstraints(1.5, 1,
                        2 * Math.PI, 1.5 * Math.PI);

        public static final HolonomicPathFollowerConfig pathFollowerConfig = new HolonomicPathFollowerConfig(
                new PIDConstants(5.0, 0, 0), // Translation PID
                new PIDConstants(5.0, 0, 0), // Rotation PID
                4.8, // Max Speed Meters per second
                Math.sqrt(Swerve.trackWidth * Swerve.trackWidth +
                        Swerve.wheelBase * Swerve.wheelBase) / 2, // Drive base radius in meters. Distance from robot center to furthest module.
                new ReplanningConfig()
        );
    }

    public static final class AutoConstants {
        public static final String ampLane = "AmpLn";
        public static final String stageLane = "StLn";
        public static final String sourceLane = "SrcLn";
    }

    public static final class GameStateConstants {

        public enum RobotCycleStatus {
            NO_NOTE_CANT_SEE_SPEAKER,
            NO_NOTE_SEES_SPEAKER,
            HAS_NOTE_CANT_SEE_SPEAKER,
            HAS_NOTE_SEES_SPEAKER
        }
    }

    public static final int SHOOTER_TOP_MOTOR_ID = 11;
    public static final int SHOOTER_BOTTOM_MOTOR_ID = 10;

    public static final double SHOOTER_MAX_ROTATIONS_PER_SECOND = 6350.0 / 60.0;
    public static final double SHOOTER_METERS_PER_ROTATION = Units.inchesToMeters(12.564); // TODO: account for any gear ratio (currently is exactly 1:1 for 4" wheel)
    public static final double shooterKS = 0.08403; // TODO: figure out what these should be!
    public static final double shooterKV = 2.2341;
    public static final double shooterKA = 0.27385;

    public static final int INTAKE_MOTOR_ID = 8; // TODO: Placeholder
    public static final int INDEXER_MOTOR_ID = 9;
    public static final double INDEXER_MAX_LIMIT = 129834.42;
    public static final double INDEXER_MIN_LIMIT = -398472938.23;

    public static final int ARM_LEFT_MOTOR_ID = 12;
    public static final int ARM_RIGHT_MOTOR_ID = 13;

    public static final int ELEVATOR_MOTOR_1_ID = 14;
    public static final int ELEVATOR_MOTOR_2_ID = 15;

    public static final double GAME_PIECE_NOTE_DIAMETER = 0.36;

    public static final double FIELD_AMP_OPENING_WIDTH = 0.6096;
    public static final double FIELD_AMP_OPENING_HEIGHT = 0.4572;
    public static final double FIELD_AMP_OPENING_HEIGHT_FROM_FLOOR = 0.6604;
    public static final double FIELD_AMP_APRIL_TAG_HEIGHT_TO_CENTER = 1.3557;

    public static final double FIELD_SPEAKER_OPENING_HEIGHT_FROM_FLOOR = 1.9812;
    public static final double FIELD_SPEAKER_OPENING_WIDTH = 1.0509;
    //public static final double FIELD_SPEAKER_OPENING_HEIGHT_AT_CENTER = ;
    public static final double FIELD_SPEAKER_OPENING_OVERHANG = 0.4572;
    public static final double FIELD_SPEAKER_APRIL_TAG_HEIGHT_OF_BOTTOM = 1.3574;
    public static final double FIELD_SPEAKER_APRIL_TAG_OFFSET_DISTANCE = 0.5668;

    public static final double FIELD_SOURCE_APRIL_TAG_HEIGHT_BOTTOM = 1.2224;
    public static final double FIELD_SOURCE_APRIL_TAG_DISTANCE_BETWEEN = 0.98425 + 0.2667;

}
