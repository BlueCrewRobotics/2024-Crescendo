package frc.robot.subsystems.noteplayer;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.lib.bluecrew.util.FieldState;
import frc.lib.bluecrew.util.RobotState;
import frc.robot.Constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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
     * Checks whether the target is within range of shooting
     *
     * @param botPose    The {@link Translation2d} of the robot
     * @param targetPose The {@link Translation2d} of the target
     * @return True if the target is within the range of the shooter
     */
    public boolean isWithinRange(Translation2d botPose, Translation2d targetPose) {
        return speedInterpolator.get(botPose.getDistance(targetPose)) <=
                speedInterpolator.get(Double.MAX_VALUE);
    }

    public void setRobotStates() {
//         TODO: Implement the rest of the logic
        switch (RobotState.getInstance().getShooterMode()) {
            case SPEAKER -> {
                RobotState.getInstance().setHasSpeakerTarget(
                        isWithinRange(PoseEstimator.getInstance().getPose().getTranslation(),
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
