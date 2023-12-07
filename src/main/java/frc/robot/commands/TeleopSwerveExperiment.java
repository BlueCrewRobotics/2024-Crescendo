package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class TeleopSwerveExperiment extends CommandBase {
    private Swerve swerve;
    private CommandXboxController driverController;

    public TeleopSwerveExperiment(Swerve swerve, CommandXboxController driverController) {
        this.swerve = swerve;
        addRequirements(swerve);

        this.driverController = driverController;
    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        double translationVal = MathUtil.applyDeadband(-driverController.getLeftY(), Constants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(-driverController.getLeftX(), Constants.stickDeadband);
        double rotationVal = 0.0;
        double angleBuffer = 5.0;
        double rotationSpeed = 0.3;

        // Rotate with D-Pad
        if (driverController.povUp().getAsBoolean()) {
            if (swerve.getGyroYaw() <= -0.0 - angleBuffer) {
                rotationVal = -rotationSpeed;
            } else if (swerve.getGyroYaw() >= 0.0 + angleBuffer) {
                rotationVal = rotationSpeed;
            }
        } else if (driverController.povRight().getAsBoolean()) {
            if (swerve.getGyroYaw() >= -90.0 && swerve.getGyroYaw() <= 90.0 - angleBuffer) {
                rotationVal = -rotationSpeed;
            } else if (swerve.getGyroYaw() < -90.0 || swerve.getGyroYaw() >= 90.0 + angleBuffer) {
                rotationVal = rotationSpeed;
            }
        } else if (driverController.povDown().getAsBoolean()) {
            if (swerve.getGyroYaw() <= 180.0 - angleBuffer && swerve.getGyroYaw() >= 0.0) {
                rotationVal = -rotationSpeed;
            } else if (swerve.getGyroYaw() >= -180 + angleBuffer && swerve.getGyroYaw() < 0.0) {
                rotationVal = rotationSpeed;
            }
        } else if (driverController.povLeft().getAsBoolean()) {
            if (swerve.getGyroYaw() >= 90.0 || swerve.getGyroYaw() <= -90.0 - angleBuffer) {
                rotationVal = -rotationSpeed;
            } else if (swerve.getGyroYaw() < 90.0 && swerve.getGyroYaw() >= -90.0 + angleBuffer) {
                rotationVal = rotationSpeed;
            }
        }
        // Rotate with joystick
        else {
            rotationVal = MathUtil.applyDeadband(-driverController.getRightX(), Constants.stickDeadband);
        }

        // Drive the swerve
        swerve.drive(
                new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed),
                rotationVal * Constants.Swerve.maxAngularVelocity,
                !driverController.leftBumper().getAsBoolean(),
                true
        );
    }
}
