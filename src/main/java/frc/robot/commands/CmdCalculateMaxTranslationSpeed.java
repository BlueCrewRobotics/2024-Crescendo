package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

public class CmdCalculateMaxTranslationSpeed extends CommandBase {

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
                new Translation2d(9.0, 0.0),
                0.0,
                true,
                true
        );
        if(timer.hasElapsed(10)) {
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
