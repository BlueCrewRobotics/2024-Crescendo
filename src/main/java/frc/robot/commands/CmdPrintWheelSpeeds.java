package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class CmdPrintWheelSpeeds extends Command {

    private SwerveSubsystem swerveSubsystem;

    public CmdPrintWheelSpeeds(SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
    }

    @Override
    public void execute() {
        System.out.println("Mod0: " + swerveSubsystem.getModuleStates()[0].speedMetersPerSecond + " Mod1: " + swerveSubsystem.getModuleStates()[1].speedMetersPerSecond + " Mod2: " + swerveSubsystem.getModuleStates()[2].speedMetersPerSecond + " Mod3: " + swerveSubsystem.getModuleStates()[3].speedMetersPerSecond);
    }
}
