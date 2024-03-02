package frc.robot.subsystems.noteplayer;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.lib.bluecrew.util.FieldState;
import frc.lib.bluecrew.util.RobotState;
import frc.robot.Constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.autos.AutoLog;
import frc.robot.subsystems.PoseEstimator;

import java.awt.geom.Line2D;
import java.util.Map;
import java.util.function.DoubleSupplier;

/**
 *
 */
public class NotePlayerSubsystem extends SubsystemBase implements Constants.NotePlayerConstants, Constants.FieldCoordinates, Constants.GameStateConstants {

    private IndexerModule indexer = new IndexerModule();
    private IntakeModule intake = new IntakeModule();
    private ArmModule arm = new ArmModule();
    private ShooterModule shooter = new ShooterModule();

    private double nextGuessAngle = TRAJECTORY_DEFAULT_INITIAL_ANGLE;

    private double shootingAngle = 90;
    private double shootingSpeed = 1;

//    private ShuffleboardTab teleopTab = Shuffleboard.getTab("Teleoperated");
//    private GenericEntry shooterSpeed =
//            teleopTab.add("Shooter Speed", 0)
//                    .getEntry();
//
//    private GenericEntry shooterAngle =
//            teleopTab.add("Shooter Angle", 0)
//                    .withWidget(BuiltInWidgets.kNumberSlider)
//                    .withProperties(Map.of("min", 0, "max", 48))
//                    .getEntry();

    private GenericEntry distanceToSpeaker = Shuffleboard.getTab("Teleoperated")
            .add("Distance To Speaker", 2d)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", 0, "max", 4))
            .getEntry();

    private final InterpolatingDoubleTreeMap speedInterpolator = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap angleInterpolator = new InterpolatingDoubleTreeMap();

    public NotePlayerSubsystem() {
        speedInterpolator.put(1.5d, 14.75d);
        speedInterpolator.put(3d, 18.2d);
        speedInterpolator.put(4d, 20.5d);

        angleInterpolator.put(1.5d, 48d);
        angleInterpolator.put(3d, 26.25d);
        angleInterpolator.put(4d, 18.5d);
    }

    public IntakeModule getIntake() {
        return intake;
    }

    public ShooterModule getShooter() {
        return shooter;
    }

    public IndexerModule getIndexer() {
        return indexer;
    }

    public ArmModule getArm() {
        return arm;
    }

    @Override
    public void periodic() {
        arm.periodic();
        setRobotStates();

//        SmartDashboard.putNumber("Angle Interpolator", angleInterpolator.get(PoseEstimator.getInstance().getPose().getTranslation().getDistance(FieldState.getInstance().getSpeakerCoords().toTranslation2d())));
//        SmartDashboard.putBoolean("Arm At Set Position", arm.isAtSetPosition());
//        SmartDashboard.putBoolean("Shooter At Set Speed", shooter.targetVelocityReached());
//        SmartDashboard.putNumber("Speed Interpolator", speedInterpolator.get(PoseEstimator.getInstance().getPose().getTranslation().getDistance(FieldState.getInstance().getSpeakerCoords().toTranslation2d())));
//        SmartDashboard.putNumber("Top Shooter Speed", shooter.getShooterTopVelocityMPS());
//        SmartDashboard.putNumber("Bottom Shooter Speed", shooter.getShooterBottomVelocityMPS());
    }

    public InterpolatingDoubleTreeMap getAngleInterpolator() {
        return angleInterpolator;
    }

    public InterpolatingDoubleTreeMap getSpeedInterpolator() {
        return speedInterpolator;
    }

    /**
     * Calculate the position of the Shooter Wheels relative to the target
     *
     * @param robotDistance The horizontal distance from the center of the robot to the target
     * @param angle         The angle of the Arm as a {@link Rotation2d}
     * @return The position of the shooter as a {@link Translation2d}, X for distance, Y for height above floor
     */
    public Translation2d calculateShooterTargetRelativePosition(double robotDistance, Rotation2d angle) {
        double verticalOffset = (-angle.getSin() * SHOOTER_ARM_LENGTH) +
                (angle.getCos() * SHOOTER_ARM_TO_WHEELS_LENGTH) +
                SHOOTER_VERTICAL_OFFSET;

        double horizontalOffset = (angle.getCos() * SHOOTER_ARM_LENGTH) +
                (angle.getSin() * SHOOTER_ARM_TO_WHEELS_LENGTH) +
                SHOOTER_HORIZONTAL_OFFSET;

        return new Translation2d(robotDistance + horizontalOffset, verticalOffset);
    }

    /**
     * Calculates the optimal shooting angle and speed for the apex of the projectile to be at the given coordinates
     * - All distances and speeds in meters
     *
     * @param robotPose    The current {@link Pose2d} of the robot in meters
     * @param targetCoords The coordinates of the target as a {@link Translation3d} in meters
     * @return The target speed and angle as a {@link Translation2d} (use the getNorm method for the speed, and the getAngle method for the angle)
     */
    public double[] calculateShootingParameters(Pose2d robotPose, Translation3d targetCoords, double initialGuessAngle) { // The brute force calculations

        // Get the horizontal distance from the robot to the target
        double robotToTargetDistance = robotPose.getTranslation().getDistance(targetCoords.toTranslation2d());

        // The shooter angle, We tested every angle incremented by 0.1 between 1 and 66, 21.7 gave
        // the lowest average number of loops before finding the optimal parameters
        Rotation2d shooterAngle = Rotation2d.fromDegrees(initialGuessAngle);

        // Pose2d representing the position of the shooter.
        // X for distance to target, Y for height above ground, Rotation2d for the angle
        Pose2d shooterPose = new Pose2d(
                calculateShooterTargetRelativePosition(robotToTargetDistance, shooterAngle),
                shooterAngle);

        // Calculate the launch speed in meters per second needed for the apex to be at the target height
        // given the height of the target and the shooter, and the angle of the shooter
        double launchSpeed = Math.sqrt(
                (targetCoords.getZ() - shooterPose.getY()) * 9.8 * 2 / Math.pow(shooterAngle.getSin(), 2));

        // Calculate the time in seconds from launching the projectile to it reaching the apex
        double timeToApex = launchSpeed * shooterAngle.getSin() / 9.8;

        // Calculate the horizontal distance in meters to the apex
        double launchDistance = launchSpeed * shooterAngle.getCos() * timeToApex;

        // Calculate the error of the expected launch distance from the target  launch distance
        double distanceError = shooterPose.getX() - launchDistance;

        int loops  = 1;
        // Do all of that over and over again until the distance error is less than a centimeter
        while ((distanceError > 0.01 || distanceError < -0.01) && loops < 10) {
            // Adjust the shooter angle according to how for off the distance is.
            // The distance error must be divided by at least 2, any less than that and the
            // angle will end up oscillating too much, the while loop will take to long, and the code will crash.
            // Tested dividing by every number between 10 and 1, 2.61 was optimal with a starting angle of 21.7
            shooterAngle = Rotation2d.fromRadians(shooterAngle.getRadians() * 1 - (distanceError / shooterPose.getX()) / 2.61);

            // Catch the angle going out of bounds.
            if (shooterAngle.getDegrees() >= 90) {
                // If the angle is >= 90 the projectile will have no positive horizontal speed
                System.out.println("*******************SHOOTER ANGLE OUT OF BOUNDS!!!!! *********************");
                shooterAngle = Rotation2d.fromDegrees(85);
            } else if (shooterAngle.getDegrees() <= 0) {
                // If the angle is <= 0 the projectile will have no positive vertical speed
                System.out.println("*******************SHOOTER ANGLE OUT OF BOUNDS!!!!! *********************");
                shooterAngle = Rotation2d.fromDegrees(5);
            }
            shooterPose = new Pose2d(
                    calculateShooterTargetRelativePosition(robotToTargetDistance, shooterAngle),
                    shooterAngle);

            launchSpeed = Math.sqrt(
                    (targetCoords.getZ() - shooterPose.getY()) * 9.8 * 2 / Math.pow(shooterAngle.getSin(), 2));

            timeToApex = launchSpeed * shooterAngle.getSin() / 9.8;

            launchDistance = launchSpeed * shooterAngle.getCos() * timeToApex;

            distanceError = shooterPose.getX() - launchDistance;

            // Increment the number of loops we've taken to make sure the code doesn't hang
            loops++;
        }

        /*
            When finding the optimal starting angle and number to divide the distance error by, we started
            with a 1-degree angle and 2 for the divisor. We then ran tests in batches of 300, with random
            robot poses (X of 4-44, Y of 5-26), and calculated the average number of times we had to calculate the shooting parameters
            including the initial calculations before the while loop. We automatically tested every combination
            of angle and divisor (0.1 increment for the angle, 0.01 increment for the divisor), and found the
            combination that resulted in the lowest average number of loops, and the lowest maximum number of
            loops. This gave us and angle of 21.7 and a divisor of 2.61, which had a maximum number of calculations
            of 6, and an average of 3.97
         */

        // Return the shooting parameters as a vector in Translation2d form

//        System.out.println("CALCULATED LAUNCH SPEED: " + launchSpeed + ", CALCULATED LAUNCH ANGLE: " + shooterAngle.getDegrees());
        return new double[] {launchSpeed * SHOOTER_TRAJECTORY_SPEED_MULTIPLIER, shooterAngle.getDegrees() * SHOOTER_TRAJECTORY_ANGLE_MULTIPLIER};
    }

    /**
     * Checks whether the stage is between the robot and the target
     *
     * @param botPose    The {@link Translation2d} of the robot
     * @param targetPose The {@link Translation2d} of the target
     * @return True if the stage is not in the way
     */
    public boolean hasLineOfSight(Translation2d botPose, Translation2d targetPose) {
        Rotation2d angleToTarget = botPose.minus(targetPose).getAngle();
        Translation2d perpendicularOffset;
        if (botPose.getY() > RED_STAGE_SPEAKER_POINT.getY()) {
            perpendicularOffset = new Translation2d(GAME_PIECE_NOTE_DIAMETER / 2, angleToTarget.minus(Rotation2d.fromDegrees(90)));
        } else {
            perpendicularOffset = new Translation2d(GAME_PIECE_NOTE_DIAMETER / 2, angleToTarget.plus(Rotation2d.fromDegrees(90)));
        }

        Line2D notePath = new Line2D.Double(botPose.getX() + perpendicularOffset.getX(), botPose.getY() + perpendicularOffset.getY(),
                targetPose.getX() + perpendicularOffset.getX(), targetPose.getY() + perpendicularOffset.getY());

        if (FieldState.getInstance().onRedAlliance()) {
            return !(notePath.intersectsLine(RED_STAGE_RIGHT) || notePath.intersectsLine(RED_CENTER_STAGE) || notePath.intersectsLine(RED_STAGE_LEFT));
        } else {
            return !(notePath.intersectsLine(BLUE_STAGE_RIGHT) || notePath.intersectsLine(BLUE_CENTER_STAGE) || notePath.intersectsLine(BLUE_STAGE_LEFT));
        }
    }

    public void setRobotStates() {
//         TODO: Implement the rest of the logic
        switch (RobotState.getInstance().getShooterMode()) {
            case SPEAKER -> {
                RobotState.getInstance().setHasSpeakerTarget(
                        hasLineOfSight(PoseEstimator.getInstance().getPose().getTranslation(),
                                FieldState.getInstance().onRedAlliance() ? RED_SPEAKER.toTranslation2d() : BLUE_SPEAKER.toTranslation2d()));
                RobotState.getInstance().setShooterStatus(
                        (arm.isAtSetPosition() && shooter.targetVelocityReached() && PoseEstimator.getInstance().getPose().getTranslation().getDistance(FieldState.getInstance().getSpeakerCoords().toTranslation2d()) < 2.7) ? ShooterStatus.READY : ShooterStatus.UNREADY);
            }
            case AMP -> {
                RobotState.getInstance().setShooterStatus(arm.isAtSetPosition() ? ShooterStatus.READY : ShooterStatus.UNREADY);
            }
            case PICKUP -> {
                RobotState.getInstance().setHasNote(intake.noteInIntake() || indexer.noteInIndexer());
                RobotState.getInstance().setShooterStatus(arm.isAtSetPosition() ? ShooterStatus.READY : ShooterStatus.UNREADY);
            }
        }

        RobotState.getInstance().setHasNote(intake.noteInIntake() || indexer.noteInIndexer());
    }

    /**
     * This is what we used for generating random robot poses for tuning the
     * calculateShootingParameters method
     *
     * @return A {@link Pose2d} with random coordinates on the blue alliance side of the field
     */
    public Pose2d generateRandomBotPose() {
        return new Pose2d(
                Units.feetToMeters(Math.random() * 23 + 4),
                Units.feetToMeters(Math.random() * 21 + 5),
                new Rotation2d()
        );
    }

    // Commands:

    public Command allStop() {
        return this.run(() -> {
            intake.stop();
            indexer.stop();
        });
    }

    public Command intakeNote() {
        return /*new AutoLog("*****\n******\n*******\n****** INTAKING NOTE ******\n*******\n******\n*****").andThen(*/(new RunCommand(() -> intake.spin(0.4))
                .until(intake::noteInIntake).andThen((
                        new RunCommand(() -> intake.spin(0.15)).withTimeout(0.4)
                                .andThen(new RunCommand(() -> intake.spin(0.1))))
                                .alongWith(pullNoteIntoIndexer())))
                .until(indexer::noteInIndexer)//.andThen(new RunCommand(() -> indexer.spin(0.5)).withTimeout(0.2))
                .finallyDo(() -> {
                    intake.stop();
                    indexer.stop();

                })
        /*)*/.withName("IntakeNote");
    }

    public Command eject() {
        return new RunCommand(
                () -> {
                    indexer.spin(-0.75);
                    intake.spin(-0.4);
                    shooter.spinPercentage(-0.03);
                }
        ).finallyDo(
                () -> {
                    indexer.stop();
                    intake.stop();
                    shooter.stop();
                }
        );
    }

    public Command pullNoteIntoIndexer() {
        return new RunCommand(
                () -> indexer.spin(1)
        ).withTimeout(0.4).andThen(new RunCommand(() -> indexer.spin(0.65))).until(indexer::noteInIndexer);
    }

    public Command feedNoteToShooter() {
        return new RunCommand(
                () -> indexer.spin(1))
                .onlyWhile(indexer::noteInIndexer)
                .andThen(indexer::stop)
                .handleInterrupt(indexer::stop);
    }

    public Command finishShooting() {
        return (new RunCommand(() -> shooter.spinMetersPerSecond(speedInterpolator.get(PoseEstimator.getInstance().getPose().getTranslation().getDistance(FieldState.getInstance().getSpeakerCoords().toTranslation2d())))).raceWith(Commands.waitSeconds(0.1))).andThen(() -> shooter.stop());
    }

    public Command prepForPickup() {
        return (new InstantCommand(
                () -> {
                    RobotState.getInstance().setShooterMode(ShooterMode.PICKUP);
                    indexer.setEnableHardLimit(true);
                })
                .alongWith(rotateArmToDegrees(ARM_PICKUP_ANGLE)));
    }

    public Command aimAndSpinUpForSpeaker() {
        return (new RunCommand(
                () -> {
                    RobotState.getInstance().setShooterMode(ShooterMode.SPEAKER);
                    indexer.setEnableHardLimit(false);
//                    shooter.spinMetersPerSecond(speedInterpolator.get(Math.abs(PoseEstimator.getInstance().getPose().getTranslation().getDistance(FieldState.getInstance().getSpeakerCoords().toTranslation2d()))));
//                    arm.rotateToDegrees(angleInterpolator.get(Math.abs(PoseEstimator.getInstance().getPose().getTranslation().getDistance(FieldState.getInstance().getSpeakerCoords().toTranslation2d()))));
                    shooter.spinMetersPerSecond(speedInterpolator.get(distanceToSpeaker.getDouble(1.5)));
                    arm.rotateToDegrees(angleInterpolator.get(distanceToSpeaker.getDouble(1.5)));
                }
        )).finallyDo(() -> {
                    shooter.stop();
                }
        ).onlyIf(indexer::noteInIndexer).onlyWhile(indexer::noteInIndexer)
                .withName("AimAndSpinUpForSpeaker");
    }

    public Command scoreNote() {
        return new InstantCommand(() -> {
            switch (RobotState.getInstance().getShooterMode()) {
                case SPEAKER -> {
                    System.out.println("Scoring Speaker");
                    CommandScheduler.getInstance().schedule(feedNoteToShooter().andThen(finishShooting()));
                }
                case AMP -> {
                    System.out.println("Scoring Amp");
                    CommandScheduler.getInstance().schedule(scoreAmp());
                }
                default -> {
                    System.out.println("Scoring Nothing");
                }
            }
        });
    }

    public Command prepForAmp() {
        return rotateArmToDegrees(ARM_AMP_ANGLE)
                .alongWith(new InstantCommand(
                        () -> {
                            RobotState.getInstance().setShooterMode(ShooterMode.AMP);
                            indexer.setEnableHardLimit(false);
                        }))
                .onlyIf(indexer::noteInIndexer)
                .onlyWhile(indexer::noteInIndexer)
                .withName("PrepForAmp");
    }

    public Command scoreAmp() {
        return new RunCommand(
                () -> {
                    shooter.spinPercentage(0.2);
                    indexer.spin(1);
                }
        ).onlyWhile(indexer::noteInIndexer).onlyIf(indexer::noteInIndexer)
                .finallyDo(() -> {
                    shooter.stop();
                    indexer.stop();
                });
    }

    public Command rotateArmToDegrees(double degrees) {
        return new InstantCommand(
                () -> arm.rotateToDegrees(degrees)
        );
    }

    public Command driveArmPercent(DoubleSupplier percent) {
        return (new RunCommand(
                () -> arm.rotatePercentOut(percent.getAsDouble()))
                .finallyDo(() -> arm.rotatePercentOut(0)));
    }
}
