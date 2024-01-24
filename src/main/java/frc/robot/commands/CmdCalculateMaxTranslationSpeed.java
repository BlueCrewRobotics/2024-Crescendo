package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDrive;

/**
 * This Command drives the {@link SwerveDrive} Drive in the positive X direction as fast as possible, then prints out the speeds of each module. DO NOT INCLUDE IN ACTUAL ROBOT CODE
 */
public class CmdCalculateMaxTranslationSpeed extends Command {

    private SwerveDrive swerveDrive;
    private Timer timer;
    private boolean isFinished;

    /**
     * @param swerveDrive The Swerve Subsystem
     */
    public CmdCalculateMaxTranslationSpeed(SwerveDrive swerveDrive) {
        this.swerveDrive = swerveDrive;

        addRequirements(swerveDrive);

        timer = new Timer();
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
                new Translation2d(10, 0.0),
                0.0,
                true,
                true
        );
        if(timer.hasElapsed(2)) {
            System.out.println("Module 0: " + swerveDrive.getModuleStates()[0].speedMetersPerSecond);
            System.out.println("Module 1: " + swerveDrive.getModuleStates()[1].speedMetersPerSecond);
            System.out.println("Module 2: " + swerveDrive.getModuleStates()[2].speedMetersPerSecond);
            System.out.println("Module 3: " + swerveDrive.getModuleStates()[3].speedMetersPerSecond);
        }
        if (timer.hasElapsed(6)) {
            isFinished = true;
            swerveDrive.drive(
                    new Translation2d(0.0, 0.0),
                    0.0,
                    true,
                    true
            );
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
