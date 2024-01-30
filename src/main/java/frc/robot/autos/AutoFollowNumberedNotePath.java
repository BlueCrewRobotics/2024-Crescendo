package frc.robot.autos;


import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import java.util.function.IntSupplier;

public class AutoFollowNumberedNotePath extends SequentialCommandGroup {

    private final String start;
    private final String end;
    private final IntSupplier num;
    public AutoFollowNumberedNotePath(String start, IntSupplier num, String end) {
        this.start = start;
        this.end = end;
        this.num = num;
    }

    @Override
    public void initialize() {
        addCommands(
                //AutoBuilder.followPath(PathPlannerPath.fromPathFile(start + num.getAsInt() + end))
                Commands.print("Following: " + start + num.getAsInt() + end)
        );
        this.m_currentCommandIndex = 0;
        if (!this.m_commands.isEmpty()) {
            ((Command)this.m_commands.get(0)).initialize();
        }
    }
}
