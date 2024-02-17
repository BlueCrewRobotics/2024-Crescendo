package frc.robot.autos;


import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import java.util.function.IntSupplier;

public class AutoFollowNumberedNotePath extends SequentialCommandGroup {

    /*
     * This class needs the IntSupplier so that it can get the note that we want when this is scheduled,
     * rather than the one we wanted when it was constructed
     */

    private final String start;
    private final String end;
    private final IntSupplier num;
    public AutoFollowNumberedNotePath(String start, IntSupplier num, String end) {
        // Save the inputs for later
        this.start = start;
        this.end = end;
        this.num = num;
    }

    @Override
    public void initialize() {
        // Follow the path to the note we want
        addCommands(
                //AutoBuilder.followPath(PathPlannerPath.fromPathFile(start + num.getAsInt() + end))
                Commands.print("Following: " + start + num.getAsInt() + end)
        );

        // THIS IS SUPER IMPORTANT, this code is needed to start the commands going,
        super.initialize();
    }
}
