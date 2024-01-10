package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class CmdCalculateMaxRotationSpeed extends Command {

    private Swerve swerve;
    private Timer timer = new Timer();
    private boolean isFinished;

    public CmdCalculateMaxRotationSpeed(Swerve swerve) {
        this.swerve = swerve;

        addRequirements(swerve);
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
                new Translation2d(0.0, 0.0).times(Constants.Swerve.maxSpeed),
                20.0,
                true,
                true
        );
        if(timer.hasElapsed(3)) {
            System.out.println("Raw Rotation Speed in Radians per Second: " + Math.toRadians(swerve.getGyroYawSpeed()));
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
