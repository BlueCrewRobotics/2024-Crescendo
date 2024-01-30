package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

public class ShootNoteIntoAmp extends Command {

    public ShootNoteIntoAmp() {}

    @Override
    public void initialize() {
        System.out.println("Shoot Note into Amp Init");
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Shoot Note into Amp End");
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
