package frc.robot.autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib.bluecrew.util.GlobalVariables;
import frc.robot.commands.IntakeNote;

public class AutoGrabFromCenter extends SequentialCommandGroup {

    private final int[] orderOfCenterNotes;
    private final String autoLane;
    private final String comingFrom;

    public AutoGrabFromCenter(int[] orderOfCenterNotes, String comingFrom,  String autoLane) {
        this.orderOfCenterNotes = orderOfCenterNotes;
        this.comingFrom = comingFrom;
        this.autoLane = autoLane;
    }

    @Override
    public void initialize() {
        addCommands(
                new FindCenterPiece(orderOfCenterNotes, comingFrom, autoLane).until(() -> GlobalVariables.getInstance().isAutoPieceIsAvailable()),
                new AutoFollowNumberedNotePath("CN", () -> GlobalVariables.getInstance().getCenterNoteIndex(), "Intake")
                        .raceWith(new IntakeNote())
                        .alongWith(new InstantCommand(() -> GlobalVariables.getInstance().setAutoPieceIsAvailable(false)))
                        .alongWith(new InstantCommand(() -> GlobalVariables.getInstance().setCenterNoteExists(
                                GlobalVariables.getInstance().getCenterNoteIndex()-1, false
                        )))
        );
        this.m_currentCommandIndex = 0;
        if (!this.m_commands.isEmpty()) {
            ((Command)this.m_commands.get(0)).initialize();
        }
    }
}
