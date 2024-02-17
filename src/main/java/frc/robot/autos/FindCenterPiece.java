package frc.robot.autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.*;
import frc.lib.bluecrew.util.FieldState;
import frc.lib.bluecrew.util.RobotState;
import frc.robot.commands.FindAndGotoNote;
import frc.robot.subsystems.noteplayer.NotePlayerSubsystem;
import frc.robot.subsystems.swervedrive.SwerveDrive;

import java.util.ArrayList;
import java.util.List;

public class FindCenterPiece extends SequentialCommandGroup {

    private final int[] orderOfCenterNotes;
    private final String autoLane;
    private final String comingFrom;
    private final NotePlayerSubsystem notePlayerSubsystem;
    private final SwerveDrive swerveDrive;

    public FindCenterPiece(int[] orderOfCenterNotes, String comingFrom, String autoLane, NotePlayerSubsystem notePlayerSubsystem, SwerveDrive swerveDrive) {
        // Save inputs for later
        this.orderOfCenterNotes = orderOfCenterNotes;
        this.comingFrom = comingFrom;
        this.autoLane = autoLane;
        this.notePlayerSubsystem = notePlayerSubsystem;
        this.swerveDrive = swerveDrive;
    }

    public final void initialize() {
        // When this command is scheduled add the commands we want to do.
        // This must be done when the command is scheduled, and is only possible
        // because this class overrides the custom SequentialCommandGroup class in this package

        // Create a list if the notes we want to get, in the order we want to get them,
        // based on the input orderOfCenterNotes and whether or note they exist
        List<Integer> centerNotesToGet = new ArrayList<>();
        for (int note : orderOfCenterNotes) {
            if (FieldState.getInstance().getCenterNotesExist()[note-1]) {
                centerNotesToGet.add(note);
            }
        }

        // If there are notes to get, do stuff
        if (!centerNotesToGet.isEmpty()) {
            // The next note to get is the first one
            int nextNote = centerNotesToGet.get(0);

            // The noteIndex is the note we are currently at, used for sending to the GlobalVariables class
            // because lambda methods can only be passed final values (idk why, ask someone else)
            final int noteIndex = nextNote;

            // first thing to do
            addCommands(
                    Commands.print("Following: " + comingFrom + "-" + autoLane + "-CL"),
                    // Follow the path from where we just scored, through the auto lane, to the center line (actually a bit behind it)
                    AutoBuilder.followPath(PathPlannerPath.fromPathFile(comingFrom + "-" + autoLane + "-CL")),
                    // Follow the path from where we stopped behind the center line to in front of the next note,
                    // using the .alongWith decorator to make its own sequential command group,
                    // which seemed to help with other things run at the right time
                    Commands.print("Following: CL-" + autoLane + "-CN" + nextNote),
                    AutoBuilder.followPath(PathPlannerPath.fromPathFile("CL-" + autoLane + "-CN" + nextNote)).andThen(

                            // But actually just print that we're doing that
                            // After we follow the paths, check if the note we are in front of is available/exists,
                            // at the same time as setting the global note index
                            Commands.print("FindingNote"),
                            new AutoFindAndGoToNote(notePlayerSubsystem, swerveDrive)
                    )
            );

            // Then do other stuff. Remember this command runs only until the CheckForPieceAvailability
            // command sets the global availability to true
            for (int i = 1; i < centerNotesToGet.size(); i++) {
                // The next note is the next note
                nextNote = centerNotesToGet.get(i);

                // The noteIndex2 is the note we are currently at, used for sending to the GlobalVariables class
                // because lambda methods can only be passed final values (idk why, ask someone else)
                final int noteIndex2 = nextNote;
                addCommands(
                        // Follow the path from the note we are currently at to the next one
                        Commands.print("Following: CN" + centerNotesToGet.get(i - 1) + "-CN" + nextNote),
                        AutoBuilder.followPath(PathPlannerPath.fromPathFile("CN-" + centerNotesToGet.get(i-1) + "-CN" + nextNote)),
                        // Set the global note index, and check for piece availability
                        new InstantCommand(() -> FieldState.getInstance().setCenterNoteIndex(noteIndex2)),
                        new AutoFindAndGoToNote(notePlayerSubsystem, swerveDrive)
                );
            }

            addCommands(
                    // Wait for it to clock through every thing (waiting 30ms, runs every 20ms),
                    // so we don't accidentally trigger this if we got the last piece
                    Commands.waitSeconds(0.03),
                    new InstantCommand(() -> FieldState.getInstance().setCenterNotesGone(true))
            );

            // THIS IS SUPER IMPORTANT, this code is needed to start the commands going,
            // this is normally done automatically with WPILib's SequentialCommandGroup,
            // but since we are using the custom one, we have to manually do this
            this.m_currentCommandIndex = 0;
            if (!this.m_commands.isEmpty()) {
                ((Command) this.m_commands.get(0)).initialize();
            }
        }
    }
}
