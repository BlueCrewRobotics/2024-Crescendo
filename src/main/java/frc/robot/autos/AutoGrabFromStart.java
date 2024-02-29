package frc.robot.autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.bluecrew.util.RobotState;
import frc.robot.Constants;
import frc.robot.commands.FindAndGotoNote;
import frc.robot.subsystems.noteplayer.NotePlayerSubsystem;
import frc.robot.subsystems.swervedrive.SwerveDrive;

public class AutoGrabFromStart extends SequentialCommandGroup {

    public AutoGrabFromStart(int noteToGet, String lastScoredIn, String autoLane, NotePlayerSubsystem notePlayerSubsystem, SwerveDrive swerveDrive) {
        // Follow the path from where we last scored to the starting note we want to get
        // until we get to the end of the path, or we pick up a note
        addCommands(
                new AutoLog("Starting Path To Note!"),
                AutoBuilder.followPath(PathPlannerPath.fromPathFile(lastScoredIn + "-" + autoLane + "-SN" + noteToGet)),
                new AutoLog("Done Following Path To Note, Starting Intake!"),
                Commands.waitUntil(() -> RobotState.getInstance().getShooterStatus() == Constants.GameStateConstants.ShooterStatus.READY),
                new FindAndGotoNote(swerveDrive).alongWith(Commands.waitUntil(RobotState.getInstance()::isNoteIsAvailable)
                        .andThen(new RunCommand(() -> notePlayerSubsystem.getIntake().spin(0.4)))).until(notePlayerSubsystem.getIntake()::noteInIntake)
                        .finallyDo(() -> notePlayerSubsystem.getIntake().stop())
        );
    }
}
