package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;

public class CmdCalculateMaxTranslationSpeed extends Command {

    private Swerve swerve;
    private Timer timer;
    private boolean isFinished;

    public CmdCalculateMaxTranslationSpeed(Swerve swerve) {
        this.swerve = swerve;

        addRequirements(swerve);

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
        swerve.drive(
                new Translation2d(5.2, 0.0),
                0.0,
                true,
                true
        );
        if(timer.hasElapsed(2)) {
            System.out.println("Module 0: " + swerve.getModuleStates()[0].speedMetersPerSecond);
            System.out.println("Module 1: " + swerve.getModuleStates()[1].speedMetersPerSecond);
            System.out.println("Module 2: " + swerve.getModuleStates()[2].speedMetersPerSecond);
            System.out.println("Module 3: " + swerve.getModuleStates()[3].speedMetersPerSecond);
        }
        if (timer.hasElapsed(6)) {
            isFinished = true;
            swerve.drive(
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
