package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveDrive;

public class CmdPrintWheelSpeeds extends Command {

    private SwerveDrive swerveDrive;

    public CmdPrintWheelSpeeds(SwerveDrive swerveDrive) {
        this.swerveDrive = swerveDrive;
    }

    @Override
    public void execute() {
        System.out.println("Mod0: " + swerveDrive.getModuleStates()[0].speedMetersPerSecond + " Mod1: " + swerveDrive.getModuleStates()[1].speedMetersPerSecond + " Mod2: " + swerveDrive.getModuleStates()[2].speedMetersPerSecond + " Mod3: " + swerveDrive.getModuleStates()[3].speedMetersPerSecond);
    }
}
