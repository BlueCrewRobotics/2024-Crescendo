package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.lib.bluecrew.util.RobotState;
import frc.robot.commands.FindAndGotoNote;
import frc.robot.subsystems.noteplayer.NotePlayerSubsystem;
import frc.robot.subsystems.swervedrive.SwerveDrive;


public class AutoGetNote extends ParallelCommandGroup {

    public AutoGetNote(SwerveDrive swerveDrive, NotePlayerSubsystem notePlayerSubsystem) {
        addCommands(
                new FindAndGotoNote(swerveDrive).until(notePlayerSubsystem.getIntake()::noteInIntake),
                Commands.waitUntil(RobotState.getInstance()::isNoteIsAvailable).andThen(notePlayerSubsystem.intakeNote())
        );
    }
}
