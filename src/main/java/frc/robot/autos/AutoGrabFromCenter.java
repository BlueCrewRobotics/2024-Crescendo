package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib.bluecrew.util.RobotState;
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
                        // End a clock cycle after we have a note
                        .raceWith(Commands.waitUntil(notePlayerSubsystem.getIntake()::noteInIntake).andThen(Commands.waitSeconds(0.025))),
                // And then set the note availability to false
                new InstantCommand(() -> RobotState.getInstance().setNoteIsAvailable(false))
        );

        // THIS IS SUPER IMPORTANT, this code is needed to start the commands going,
        super.initialize();
    }
}
