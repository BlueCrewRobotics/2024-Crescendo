package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * This Command drives the {@link SwerveSubsystem} Drive in the positive X direction as fast as possible, then prints out the speeds of each module. DO NOT INCLUDE IN ACTUAL ROBOT CODE
 */
public class CmdCalculateMaxTranslationSpeed extends Command {

    private SwerveSubsystem swerveSubsystem;
    private Timer timer;
    private boolean isFinished;

    /**
     * @param swerveSubsystem The Swerve Subsystem
     */
    public CmdCalculateMaxTranslationSpeed(SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;

        addRequirements(swerveSubsystem);

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
        swerveSubsystem.drive(
                new Translation2d(10, 0.0),
                0.0,
                true,
                true
        );
        if(timer.hasElapsed(2)) {
            System.out.println("Module 0: " + swerveSubsystem.getModuleStates()[0].speedMetersPerSecond);
            System.out.println("Module 1: " + swerveSubsystem.getModuleStates()[1].speedMetersPerSecond);
            System.out.println("Module 2: " + swerveSubsystem.getModuleStates()[2].speedMetersPerSecond);
            System.out.println("Module 3: " + swerveSubsystem.getModuleStates()[3].speedMetersPerSecond);
        }
        if (timer.hasElapsed(6)) {
            isFinished = true;
            swerveSubsystem.drive(
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
