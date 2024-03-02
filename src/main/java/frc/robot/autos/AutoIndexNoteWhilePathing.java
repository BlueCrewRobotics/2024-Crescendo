package frc.robot.autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.noteplayer.NotePlayerSubsystem;

public class AutoIndexNoteWhilePathing extends ParallelCommandGroup {

    public AutoIndexNoteWhilePathing(NotePlayerSubsystem notePlayerSubsystem, String pathName, PathConstraints constraints) {
        addCommands(
//                new AutoLog("Starting Indexing while driving!"),
                AutoBuilder.buildAuto(pathName),
                notePlayerSubsystem.intakeNote()
        );
    }
}
