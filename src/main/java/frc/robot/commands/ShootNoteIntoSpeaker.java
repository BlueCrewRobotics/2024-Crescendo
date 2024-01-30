package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

public class ShootNoteIntoSpeaker extends Command {

    public ShootNoteIntoSpeaker() {}

    @Override
    public void initialize() {
        System.out.println("Shoot Note Into Speaker Init");
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Shoot Note Into Speaker End");
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
