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
                    Commands.print("Following: CL-" + autoLane + "-CN" + nextNote),
                    AutoBuilder.followPath(PathPlannerPath.fromPathFile("CL-" + autoLane + "-CN" + nextNote)).andThen(
                            // After we follow the paths, try to grab the note in front of us, unless it doesn't exist
                            Commands.print("FindingNote"),
                            new FindAndGotoNote(swerveDrive).until(notePlayerSubsystem.getIntake()::noteInIntake)
                                    .alongWith(Commands.waitUntil(RobotState.getInstance()::isNoteIsAvailable).withTimeout(0.06)
                                            .andThen(notePlayerSubsystem.intakeNote().onlyIf(RobotState.getInstance()::isNoteIsAvailable)))
                    )
            );

            // Then do other stuff. This command is interrupted as soon as there's a note in the intake
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
                        // Try to grab the note in front of us
                        new FindAndGotoNote(swerveDrive).until(notePlayerSubsystem.getIntake()::noteInIntake)
                                .alongWith(Commands.waitUntil(RobotState.getInstance()::isNoteIsAvailable).withTimeout(0.06)
                                        .andThen(notePlayerSubsystem.intakeNote().onlyIf(RobotState.getInstance()::isNoteIsAvailable))),
                        // Either we got it, or it wasn't there, so save that it's gone
                        new InstantCommand(() -> FieldState.getInstance().setCenterNoteExists(noteIndex2, false))
                );
            }
        }
        // Lastly, set that all the notes we want are gone, this command will be interrupted before this if we get a note
        addCommands(
                new InstantCommand(() -> FieldState.getInstance().setCenterNotesGone(true))
        );

        // THIS IS SUPER IMPORTANT, this code is needed to start the commands going,
        super.initialize();
    }
}
