package frc.robot.autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.*;
import frc.lib.bluecrew.util.GlobalVariables;

import java.util.ArrayList;
import java.util.List;

public class FindCenterPiece extends SequentialCommandGroup {

    private final int[] orderOfCenterNotes;
    private final String autoLane;
    private final String comingFrom;

    public FindCenterPiece(int[] orderOfCenterNotes, String comingFrom,  String autoLane) {
        this.orderOfCenterNotes = orderOfCenterNotes;
        this.comingFrom = comingFrom;
        this.autoLane = autoLane;
    }

    public final void initialize() {
        List<Integer> centerNotesToGet = new ArrayList<>();
        for (int note : orderOfCenterNotes) {
            if (GlobalVariables.getInstance().getCenterNotesExist()[note-1]) {
                centerNotesToGet.add(note);
            }
        }

        if (!centerNotesToGet.isEmpty()) {
            int nextNote = centerNotesToGet.get(0);

            int noteIndex = nextNote;
            // TODO: add logic for if we've just scored in the Amp rather than speaker
            addCommands(
//                AutoBuilder.followPath(PathPlannerPath.fromPathFile(comingFrom + "-" + autoLane + "-CL")),
//                AutoBuilder.followPath(PathPlannerPath.fromPathFile("CL-" + autoLane + "-CN" + nextNote)).alongWith(
                    Commands.print("Following: " + comingFrom + "-" + autoLane + "-CL"),
                    Commands.print("Following: CL-" + autoLane + "-CN" + nextNote).andThen(
                            new CheckForPieceAvailability()).alongWith(
                            new InstantCommand(() -> GlobalVariables.getInstance().setCenterNoteIndex(noteIndex))
                    )
            );

            for (int i = 1; i < centerNotesToGet.size(); i++) {
                nextNote = centerNotesToGet.get(i);
                int noteIndex2 = nextNote;
                addCommands(
                        //AutoBuilder.followPath(PathPlannerPath.fromPathFile("CN-" + centerNotesToGet.get(i-1) + "-CN" + nextNote)),
                        Commands.print("Following: CN" + centerNotesToGet.get(i - 1) + "-CN" + nextNote),
                        new InstantCommand(() -> GlobalVariables.getInstance().setCenterNoteIndex(noteIndex2)),
                        new CheckForPieceAvailability()
                );
            }

            this.m_currentCommandIndex = 0;
            if (!this.m_commands.isEmpty()) {
                ((Command) this.m_commands.get(0)).initialize();
            }
        }
    }
}
