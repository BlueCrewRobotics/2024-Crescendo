package frc.robot.autos;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoLog extends Command {

    private final String log;

    public AutoLog(String log) {
        this.log = log;
    }

    @Override
    public void initialize() {
        DataLogManager.log(log);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
