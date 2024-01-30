package frc.robot.autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.commands.IntakeNote;

public class AutoGrabFromStart extends ParallelRaceGroup {

    public AutoGrabFromStart(int noteToGet, String autoLane) {
        addCommands(
                //AutoBuilder.followPath(PathPlannerPath.fromPathFile("Sp-" + autoLane + "-SN" + noteToGet)),
                Commands.print("Following: Sp-" + autoLane + "-SN" + noteToGet),
                new IntakeNote()
        );
    }
}
