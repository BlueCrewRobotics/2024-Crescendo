package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib.bluecrew.util.FieldState;
import frc.lib.bluecrew.util.RobotState;
import frc.robot.commands.IntakeNote;
import frc.robot.subsystems.noteplayer.NotePlayerSubsystem;
import frc.robot.subsystems.swervedrive.SwerveDrive;

public class AutoGrabFromCenter extends SequentialCommandGroup {

    private final int[] orderOfCenterNotes;
    private final String autoLane;
    private final String comingFrom;
    private final NotePlayerSubsystem notePlayerSubsystem;
    private final SwerveDrive swerveDrive;

    public AutoGrabFromCenter(int[] orderOfCenterNotes, String comingFrom, String autoLane, NotePlayerSubsystem notePlayerSubsystem, SwerveDrive swerveDrive) {
        // Save the inputs for later
        this.orderOfCenterNotes = orderOfCenterNotes;
        this.comingFrom = comingFrom;
        this.autoLane = autoLane;
        this.notePlayerSubsystem = notePlayerSubsystem;
        this.swerveDrive = swerveDrive;
    }

    @Override
    public void initialize() {
        // When this command is scheduled add the commands we want to do.
        // This must be done when the command is scheduled, and is only possible
        // because this class overrides the custom SequentialCommandGroup class in this package
        addCommands(
                // Look for a note until we see that one is available
                new FindCenterPiece(orderOfCenterNotes, comingFrom, autoLane, notePlayerSubsystem, swerveDrive)
                        // End half a second after we have a note
                        .raceWith(Commands.waitUntil(notePlayerSubsystem.getIndexer()::noteInIndexer)),
                // Follow the path to the note we are in front of until the path ends, or we pick up a note,
                // but only if AutoPieceIsAvailable is true
//                (new AutoFollowNumberedNotePath("CN", () -> FieldState.getInstance().getCenterNoteIndex(), "Intake")
//                        // Race with IntakeNote command
//                        .raceWith(new IntakeNote())
                        // At the same time, set the piece availability to false
                        new InstantCommand(() -> RobotState.getInstance().setNoteIsAvailable(false))
                        // And set that the note we are in front of no longer exists (because we are picking it up
                        .alongWith(new InstantCommand(() -> FieldState.getInstance().setCenterNoteExists(
                                FieldState.getInstance().getCenterNoteIndex()-1, false)))
                // Only if a piece is available
                .onlyIf(() -> RobotState.getInstance().isNoteIsAvailable())
        );

        // THIS IS SUPER IMPORTANT, this code is needed to start the commands going,
        super.initialize();
    }
}
