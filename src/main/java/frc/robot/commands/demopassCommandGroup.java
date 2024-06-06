package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.noteplayer.NotePlayerSubsystem;
import frc.robot.subsystems.swervedrive.SwerveDrive;

public class demopassCommandGroup extends SequentialCommandGroup {
    public demopassCommandGroup(SwerveDrive swervDrive, NotePlayerSubsystem notePlayer) {
        // TODO: Add your sequential commands in the super() call, e.g.
        //           super(new OpenClawCommand(), new MoveArmCommand());
        super(notePlayer.demothrow(), new demoturn(swervDrive), new FindAndGotoNote(swervDrive));
    }
}