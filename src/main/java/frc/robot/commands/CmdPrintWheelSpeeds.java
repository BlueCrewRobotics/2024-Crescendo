package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;

public class CmdPrintWheelSpeeds extends Command {

    private Swerve swerve;

    public CmdPrintWheelSpeeds(Swerve swerve) {
        this.swerve = swerve;
    }

    @Override
    public void execute() {
        System.out.println("Mod0: " + swerve.getModuleStates()[0].speedMetersPerSecond + " Mod1: " + swerve.getModuleStates()[1].speedMetersPerSecond + " Mod2: " + swerve.getModuleStates()[2].speedMetersPerSecond + " Mod3: " + swerve.getModuleStates()[3].speedMetersPerSecond);
    }
}
