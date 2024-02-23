package frc.robot.autos;

import frc.lib.bluecrew.util.RobotState;
import frc.robot.commands.FindAndGotoNote;
import frc.robot.subsystems.noteplayer.NotePlayerSubsystem;
import frc.robot.subsystems.swervedrive.SwerveDrive;


public class AutoFindAndGoToNote extends FindAndGotoNote {

    public AutoFindAndGoToNote(NotePlayerSubsystem notePlayerSubsystem, SwerveDrive swerveDrive) {
        super(notePlayerSubsystem, swerveDrive);
    }

    @Override
    public boolean isFinished() {
        return !RobotState.getInstance().isNoteIsAvailable();
    }

}
