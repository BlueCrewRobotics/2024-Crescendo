package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrive;

public class TeleopSwerveExperiment extends Command {
    private SwerveDrive swerveDrive;
    private CommandXboxController driverController;

    public TeleopSwerveExperiment(SwerveDrive swerveDrive, CommandXboxController driverController) {
        this.swerveDrive = swerveDrive;
        addRequirements(swerveDrive);

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
        if (driverController.getHID().getPOV() == 0) {
            if (swerveDrive.getGyroYaw().getDegrees() <= -0.0 - angleBuffer) {
                rotationVal = -rotationSpeed;
            } else if (swerveDrive.getGyroYaw().getDegrees() >= 0.0 + angleBuffer) {
                rotationVal = rotationSpeed;
            }
        } else if (driverController.getHID().getPOV() == 90) {
            if (swerveDrive.getGyroYaw().getDegrees() >= -90.0 && swerveDrive.getGyroYaw().getDegrees() <= 90.0 - angleBuffer) {
                rotationVal = -rotationSpeed;
            } else if (swerveDrive.getGyroYaw().getDegrees() < -90.0 || swerveDrive.getGyroYaw().getDegrees() >= 90.0 + angleBuffer) {
                rotationVal = rotationSpeed;
            }
        } else if (driverController.getHID().getPOV() == 180) {
            if (swerveDrive.getGyroYaw().getDegrees() <= 180.0 - angleBuffer && swerveDrive.getGyroYaw().getDegrees() >= 0.0) {
                rotationVal = -rotationSpeed;
            } else if (swerveDrive.getGyroYaw().getDegrees() >= -180 + angleBuffer && swerveDrive.getGyroYaw().getDegrees() < 0.0) {
                rotationVal = rotationSpeed;
            }
        } else if (driverController.getHID().getPOV() == 270) {
            if (swerveDrive.getGyroYaw().getDegrees() >= 90.0 || swerveDrive.getGyroYaw().getDegrees() <= -90.0 - angleBuffer) {
                rotationVal = -rotationSpeed;
            } else if (swerveDrive.getGyroYaw().getDegrees() < 90.0 && swerveDrive.getGyroYaw().getDegrees() >= -90.0 + angleBuffer) {
                rotationVal = rotationSpeed;
            }
        }
        // Rotate with joystick
        else {
            rotationVal = MathUtil.applyDeadband(-0.6 * driverController.getRightX(), Constants.stickDeadband);
        }

        if(driverController.getHID().getRawButton(XboxController.Button.kB.value)) {
            translationVal *= 0.5;
            strafeVal *= 0.5;
        }

        //System.out.println("rotationVal = " + rotationVal + " translationVal = " + translationVal + " strafeVal = " + strafeVal);

        // Drive the swerve
        swerveDrive.drive(
                new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed),
                rotationVal * Constants.Swerve.maxAngularVelocity,
                !driverController.leftBumper().getAsBoolean(),
                true
        );
    }
}
