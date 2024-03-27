package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.VisionModule;
import frc.robot.subsystems.swervedrive.SwerveDrive;
import org.photonvision.PhotonCamera;


public class demoturn extends Command {

    private SwerveDrive swerveDrive;
    private PhotonCamera noteCamera = VisionModule.getInstance().getNotesIndexerCamera();

    public demoturn(SwerveDrive swerveDrive) {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(swerveDrive);
        this.swerveDrive = swerveDrive;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        swerveDrive.drive(new Translation2d(),1d, false, false);
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return noteCamera.getLatestResult().hasTargets();
    }

    @Override
    public void end(boolean interrupted) {

    }
}
