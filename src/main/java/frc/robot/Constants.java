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
public interface Constants {

    /**
     * All the driver controls constants
     */
    interface DriverControls {
        double stickDeadband = 0.1;
        double swerveSensitivityExponent = 1.8; // Should to be >= 1
        double swerveSpeedMultiplier = 0.7; // Should be <= 1
        double swerveRotationMultiplier = 0.6; // Should be <= 1
    }
    /**
     * All the Swerve Subsystem constants
     */
    interface Swerve {

        boolean invertGyro = true; // Always ensure Gyro is CCW+ CW-

        COTSTalonFXSwerveConstants chosenModule =
                COTSTalonFXSwerveConstants.SDS.MK4i.Falcon500(COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios.L2);

        /* Drivetrain Constants */
        double trackWidth = Units.inchesToMeters(20.5);
        double wheelBase = Units.inchesToMeters(20.5);
        double wheelCircumference = chosenModule.wheelCircumference;

        /* Swerve Kinematics
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
        SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
                new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        double driveGearRatio = chosenModule.driveGearRatio;
        double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        InvertedValue angleMotorInvert = chosenModule.angleMotorInvert;
        InvertedValue driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        SensorDirectionValue cancoderInvert = chosenModule.cancoderInvert;

        /* Swerve Current Limiting */
        int angleCurrentLimit = 25;
        int angleCurrentThreshold = 40;
        double angleCurrentThresholdTime = 0.1;
        boolean angleEnableCurrentLimit = true;

        int driveCurrentLimit = 35;
        int driveCurrentThreshold = 60;
        double driveCurrentThresholdTime = 0.1;
        boolean driveEnableCurrentLimit = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        double openLoopRamp = 0.25;
        double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        double angleKP = 70;//chosenModule.angleKP;
        double angleKI = chosenModule.angleKI;
        double angleKD = chosenModule.angleKD;

        /* Drive Motor PID Values */
        double driveKP = 0.1;
        double driveKI = 0.0;
        double driveKD = 0.0;

        /* Drive Motor Characterization Values From SYSID */
        double driveKS = 0.08403;
        double driveKV = 2.2341;
        double driveKA = 0.27385;

        /* Swerve Profiling Values */
        /**
         * Meters per Second
         */
        double maxSpeed = 4.9;
        /**
         * Radians per Second
         */
        double maxAngularVelocity = 12.1725; //TODO: This is theoretical!

        /* Neutral Modes */
        NeutralModeValue angleNeutralMode = NeutralModeValue.Brake;
        NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

        /* Module Specific Constants */

        /**
         * Constants for the Front Left Module - Module 0
         */
        interface Mod0 {
            int driveMotorID = 0;
            int angleMotorID = 4;
            int canCoderID = 0;
            Rotation2d angleOffset = Rotation2d.fromDegrees(138.58);
            SwerveModuleConstants constants =
                    new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /**
         * Constants for the Front Right Module - Module 1
         */
        interface Mod1 {
            int driveMotorID = 1;
            int angleMotorID = 5;
            int canCoderID = 1;
            Rotation2d angleOffset = Rotation2d.fromDegrees(204.84);
            SwerveModuleConstants constants =
                    new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /**
         * Constants for the Back Left Module - Module 2
         */
        interface Mod2 {
            int driveMotorID = 2;
            int angleMotorID = 6;
            int canCoderID = 2;
            Rotation2d angleOffset = Rotation2d.fromDegrees(115.35);
            SwerveModuleConstants constants =
                    new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /**
         * Constants for the Back Right Module - Module 3
         */
        interface Mod3 {
            int driveMotorID = 3;
            int angleMotorID = 7;
            int canCoderID = 3;
            Rotation2d angleOffset = Rotation2d.fromDegrees(213.43);
            SwerveModuleConstants constants =
                    new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
    }

    interface PhotonVision {

        String noteCamera = "noteCamera";
        String aprilTagCamera1 = "aprilTagCamera1";
        String aprilTagCamera2 = "aprilTagCamera2";
        // Cam mounted facing forward, half a meter forward of center, half a meter up from center.
        Transform3d robotToTagCam1 =
                new Transform3d(new Translation3d(0.0, 0.0, 0.0), new Rotation3d(Math.toRadians(32), 0, 0));
        Transform3d robotToTagCam2 =
                new Transform3d(new Translation3d(0.0, 0.0, 0.0), new Rotation3d(Math.toRadians(32), 0, 0));

        // The layout of the AprilTags on the field
        AprilTagFieldLayout tagLayout =
                AprilTagFields.kDefaultField.loadAprilTagLayoutField();

        // The standard deviations of our vision estimated poses, which affect correction rate
        // (Fake values. Experiment and determine estimation noise on an actual robot.)
        Matrix<N3, N1> singleTagStdDevs = VecBuilder.fill(4, 4, 8);
        Matrix<N3, N1> multiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
    }

    /**
     * Contains all the Constants used by Path Planner
     */
    interface PathPlannerConstants {

        PathConstraints pathConstraints =
                new PathConstraints(1.5, 1,
                        2 * Math.PI, 1.5 * Math.PI);

        HolonomicPathFollowerConfig pathFollowerConfig = new HolonomicPathFollowerConfig(
                new PIDConstants(5.0, 0, 0), // Translation PID
                new PIDConstants(5.0, 0, 0), // Rotation PID
                4.8, // Max Speed Meters per second
                Math.sqrt(Swerve.trackWidth * Swerve.trackWidth +
                        Swerve.wheelBase * Swerve.wheelBase) / 2, // Drive base radius in meters. Distance from robot center to furthest module.
                new ReplanningConfig()
        );
    }

    interface AutoConstants {
        String ampLane = "AmpLn";
        String stageLane = "StLn";
        String sourceLane = "SrcLn";
    }

    interface GameStateConstants {

        enum RobotCycleStatus {
            NO_NOTE_CANT_SEE_SPEAKER,
            NO_NOTE_SEES_SPEAKER,
            HAS_NOTE_CANT_SEE_SPEAKER,
            HAS_NOTE_SEES_SPEAKER
        }
    }

    interface ShooterConstants {

        enum ShooterStatus {
            PRIMED,
            UNREADY
        }

        enum ShooterMode {
            PICKUP,
            SPEAKER,
            AMP,
            TRAP
        }

        int SHOOTER_TOP_MOTOR_ID = 10;
        int SHOOTER_BOTTOM_MOTOR_ID = 11;

        double SHOOTER_MAX_ROTATIONS_PER_SECOND = 6350.0 / 60.0;
        double SHOOTER_METERS_PER_ROTATION = Units.inchesToMeters(12.564); // TODO: account for any gear ratio (currently is exactly 1:1 for 4" wheel)
        double shooterKS = 0.01; // TODO: figure out what these should be!
        double shooterKV = 0.348;
        double shooterKA = 0.15;
    }

    interface NotePlayerConstants {
        int INTAKE_MOTOR_ID = 12;
        int INDEXER_MOTOR_ID = 3;

        int ARM_LEFT_MOTOR_ID = 1;
        int ARM_RIGHT_MOTOR_ID = 2;
        int ARM_CAN_CODER_ID = 4;
        float ARM_MAX_LIMIT = 11.0f;
        float ARM_MIN_LIMIT = -11.77f;

        double SHOOTER_HORIZONTAL_OFFSET = Units.inchesToMeters(-6);
        double SHOOTER_VERTICAL_OFFSET = Units.inchesToMeters(19.25);
        double SHOOTER_ARM_LENGTH = Units.inchesToMeters(13);
        double SHOOTER_ARM_TO_WHEELS_LENGTH = Units.inchesToMeters(8);
    }

    interface ElevatorConstants {
        int ELEVATOR_MOTOR_1_ID = 8;
        int ELEVATOR_MOTOR_2_ID = 9;
    }

    interface Misc {
        // blinkin runs on pwm, not CAN
        int BLINKIN_PORT = 9;
    }

    double GAME_PIECE_NOTE_DIAMETER = 0.36;

    double FIELD_AMP_OPENING_WIDTH = 0.6096;
    double FIELD_AMP_OPENING_HEIGHT = 0.4572;
    double FIELD_AMP_OPENING_HEIGHT_FROM_FLOOR = 0.6604;
    double FIELD_AMP_APRIL_TAG_HEIGHT_TO_CENTER = 1.3557;

    double FIELD_SPEAKER_OPENING_HEIGHT_FROM_FLOOR = 1.9812;
    double FIELD_SPEAKER_OPENING_WIDTH = 1.0509;
    //double FIELD_SPEAKER_OPENING_HEIGHT_AT_CENTER = ;
    double FIELD_SPEAKER_OPENING_OVERHANG = 0.4572;
    double FIELD_SPEAKER_APRIL_TAG_HEIGHT_OF_BOTTOM = 1.3574;
    double FIELD_SPEAKER_APRIL_TAG_OFFSET_DISTANCE = 0.5668;

    double FIELD_SOURCE_APRIL_TAG_HEIGHT_BOTTOM = 1.2224;
    double FIELD_SOURCE_APRIL_TAG_DISTANCE_BETWEEN = 0.98425 + 0.2667;

}
