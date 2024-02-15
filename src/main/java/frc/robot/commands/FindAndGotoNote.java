package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.VisionModule;
import frc.robot.subsystems.noteplayer.NotePlayerSubsystem;
import frc.robot.subsystems.swervedrive.SwerveDrive;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;


public class FindAndGotoNote extends Command {
    private final NotePlayerSubsystem notePlayerSubsystem;
    private final SwerveDrive swerveDrive;

    private boolean finished = false;

    private PhotonCamera notesCamera;

    public FindAndGotoNote(NotePlayerSubsystem notePlayerSubsystem, SwerveDrive swerveDrive) {
        this.notePlayerSubsystem = notePlayerSubsystem;
        this.swerveDrive = swerveDrive;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.notePlayerSubsystem, this.swerveDrive);

        notesCamera = VisionModule.getInstance().getNotesIndexerCamera();
    }

    Command defDriveCommand;

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {


        // Define rotation needed to center-in on note
        double neededRotation = 0.0d; // in degrees
        double neededSpeed = 0.0d; // 1.0 == 100%
        final double heading = swerveDrive.getHeading().getDegrees();

        System.out.println("Looking for a note.");

        PhotonPipelineResult pipelineResult = notesCamera.getLatestResult();

        // Check if limelight has found a target
        if (pipelineResult.hasTargets()) {
            System.out.println("Note found!  --");
            PhotonTrackedTarget target = pipelineResult.getBestTarget();

            System.out.println("Pitch: " + target.getPitch());
            System.out.println("Yaw: " + target.getYaw());

            /*
                Observed pitches and yaws from the indexer camera with a note place at the following distances

                Pitches
                =======
                0 ft in front = -23.5 pitch
                1 ft in front = -0.90
                2 ft in front = 9.5
                3 ft in front = 15.0
                4 ft in front = 18.4
                5 ft in front = 21.3

                Yaws
                ======
                0 ft to right = 0 yaw (at 18" distance away from front)
                1 ft to right = 25.0
                2 ft to right = 43.25
                2 ft to right = 24.9 (at 36" distance away from front)

             */


            // Select the tollerence (in degrees) of the angle toward the target based on distance (in feet, approximated to pitch)
            int angleTolerance = 2;
            double angleOffset = 3;


            if(target.getPitch() < -24) {
                angleTolerance = 1;
                angleOffset = 5;
                neededSpeed = 0.00;
            }
            else if(target.getPitch() < 0) {
                angleTolerance = 1;
                angleOffset = 8.5;
                neededSpeed = 0.01;
            }
            else if(target.getPitch() < 15) {
                angleTolerance = 1;
                angleOffset = 6.5;
                neededSpeed = 0.025;
            }
            else if(target.getPitch() > 15) {
                angleTolerance = 2;
                angleOffset = 2;
                neededSpeed = 0.05;
            }


            if(target.getYaw() < -angleTolerance) {
                System.out.println("moving left to target!");
                neededRotation = angleOffset;
            }
            else if(target.getYaw() > angleTolerance) {
                System.out.println("moving right to target!");
                neededRotation = -angleOffset;
            }
        }
        else {
            System.out.println("No Note in view...");
            neededSpeed = 0.0;
            neededRotation = 0.0;
        }

        final double speed = neededSpeed;
        final double rotation = neededRotation;

        // forward/back (translation), left/right (strafe), slowness, target heading in degrees, robot centric
        swerveDrive.driveSwerveDriveAndRotateToAngle(() -> speed, () -> 0.0 , () -> 0.0, () -> heading + rotation, () -> true);


    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {
    }
}
