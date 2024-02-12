package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.bluecrew.util.RobotState;

public class CheckForPieceAvailability extends Command {

    // Check if we want to grab the note in front of us. Does not currently do anything

    // TODO: this needs to check if there's a note in front of the robot, and if the robot should grab it. If the robot should grab it, set the pieceAvailability in GlobalVariables to true, else set the GlobalVariables note existence to false

    public CheckForPieceAvailability() {}

    @Override
    public void initialize() {
        System.out.println("Check for piece availability init");
        RobotState.getInstance().setAutoPieceIsAvailable(true);
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Check for piece availability end");
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
