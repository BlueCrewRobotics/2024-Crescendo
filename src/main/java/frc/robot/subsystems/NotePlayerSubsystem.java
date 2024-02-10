package frc.robot.subsystems;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

import java.util.ArrayList;
import java.util.List;

/**
 *
 */
public class NotePlayerSubsystem extends SubsystemBase {

    private IndexerModule indexer = new IndexerModule();
    private IntakeModule intake = new IntakeModule();
    private ArmModule arm = new ArmModule();
    private ShooterModule shooter = new ShooterModule();

    private List<Integer> loopCounters = new ArrayList<>();
    private List<Long> loopTimes = new ArrayList<>();

    public NotePlayerSubsystem() {

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

    @Override
    public void periodic() {
        if (intake.noteInIntake()) {
            System.out.println("I SEE A NOTE WOWIIEEEEE!!!");
        }
    }

    /**
     * Calculate the position of the Shooter Wheels relative to the target
     *
     * @param robotDistance The horizontal distance from the center of the robot to the target
     * @param angle The angle of the Arm as a {@link Rotation2d}
     * @return The position of the shooter as a {@link Translation2d}, X for distance, Y for height above floor
     */
    public Translation2d calculateShooterTargetRelativePosition(double robotDistance, Rotation2d angle) {
        double verticalOffset = (-angle.getSin() * Constants.SHOOTER_ARM_LENGTH) +
                (angle.getCos() * Constants.SHOOTER_ARM_TO_WHEELS_LENGTH) +
                Constants.SHOOTER_VERTICAL_OFFSET;

        double horizontalOffset = (angle.getCos() * Constants.SHOOTER_ARM_LENGTH) +
                (angle.getSin() * Constants.SHOOTER_ARM_TO_WHEELS_LENGTH) +
                Constants.SHOOTER_HORIZONTAL_OFFSET;

        return new Translation2d(robotDistance + horizontalOffset, verticalOffset);
    }

    /**
     * Calculates the optimal shooting angle and speed for the apex of the projectile to be at the given coordinates
     *  - All distances and speeds in meters
     * @param robotPose The current {@link Pose2d} of the robot in meters
     * @param targetCoords The coordinates of the target as a {@link Translation3d} in meters
     * @return The target speed and angle as a {@link Translation2d} (use the getNorm method for the speed, and the getAngle method for the angle)
     */
    public Translation2d calculateShootingParameters(Pose2d robotPose, Translation3d targetCoords) { // The brute force calculations

        // Get the horizontal distance from the robot to the target
        double robotToTargetDistance = robotPose.getTranslation().getDistance(targetCoords.toTranslation2d());

        // The shooter angle, We tested every angle divisible by five between 45 and 10, 25 gave
        // the lowest average number of loops before finding the optimal parameters
        Rotation2d shooterAngle = Rotation2d.fromDegrees(25);

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

        // Do all of that over and over again until the distance error is less than a centimeter
        while (distanceError > 0.01 || distanceError < -0.01) {
            // Adjust the shooter angle according to how for off the distance is.
            // The distance error must be divided by at least 2, any less than that and the
            // angle will end up oscillating too much, the while loop will take to long, and the code will crash.
            // Tested dividing by every number between 10 and 1, 2.5 was optimal
            shooterAngle = Rotation2d.fromRadians(shooterAngle.getRadians() * 1 - (distanceError / shooterPose.getX()) / 2.5);

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
        }

        /*
            When finding the optimal starting angle and number to divide the distance error by, we started
            with a 45-degree angle and 10 for the divisor. We then ran tests in batches of 100, with random
            robot poses, and calculated the average number of times we had to calculate the shooting parameters
            including the initial calculations before the while loop.
            It started at an average of about 23 times, changing the initial angle to 25 dropped that to about 21
            tuning the divisor to 2.5 brought it way down to an average of 3.7 calculations.
            To be fully sure that there weren't any wierd edge cases, we ran through 50000 (probably overkill)
            trials from randomly generated robot poses. The maximum number of calculations it ever took was 6
         */

        // Return the shooting parameters as a vector in Translation2d form
        return new Translation2d(launchSpeed, shooterAngle);
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

    public Command intakeNote() {
        return this.startEnd(
                        () -> intake.spin(0.25),
                        () -> intake.stopSpinning())
                .until(() -> intake.noteInIntake());
    }

    public Command runIndexer() {
        return null; /* this.startEnd(
                () -> indexer.
        )*/
    }

    public Command aimAtTarget() {
        Translation3d targetCoords = new Translation3d(
                Units.inchesToMeters(-1.5),
                Units.inchesToMeters(218.42),
                Units.inchesToMeters(80)
        );

        return new InstantCommand(() -> this.calculateShootingParameters(generateRandomBotPose(), targetCoords));
    }
}
