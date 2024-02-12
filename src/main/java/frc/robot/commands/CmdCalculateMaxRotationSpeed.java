package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.swervedrive.SwerveDrive;

/**
 * This command directs the {@link SwerveDrive} to rotate much faster than it is actually capable of, and then prints out the angular velocity according to the NavX. DO NOT INCLUDE IN ACTUAL ROBOT CODE
 */
public class CmdCalculateMaxRotationSpeed extends Command {

    private SwerveDrive swerveDrive;
    private Timer timer = new Timer();
    private boolean isFinished;

    /**
     * @param swerveDrive The Swerve Subsystem
     */
    public CmdCalculateMaxRotationSpeed(SwerveDrive swerveDrive) {
        this.swerveDrive = swerveDrive;

        addRequirements(swerveDrive);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        isFinished = false;
    }

    @Override
    public void execute() {
        swerveDrive.drive(
                new Translation2d(0.0, 0.0).times(Constants.Swerve.maxSpeed),
                20.0,
                true,
                true
        );
        if(timer.hasElapsed(3)) {
            System.out.println("Raw Rotation Speed in Radians per Second: " + Math.toRadians(swerveDrive.getGyroYawSpeed()));
            isFinished = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }
}
