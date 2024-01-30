package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.bluecrew.util.GlobalVariables;

public class IntakeNote extends Command {

    /* Todo: this needs to end when the note has been fully picked up (inside Indexer?)
     * also should maybe be a sequential command, to move the shooter arm and shooter into position and spin up the indexer and intake
     */

    public IntakeNote() {}

    @Override
    public void initialize() {
        System.out.println("Intake Note Init");
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Intake Note End");
        GlobalVariables.getInstance().setAutoPieceIsAvailable(false);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
