package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;


import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.bluecrew.pathplanner.CustomAutoBuilder;
import frc.lib.math.Conversions;
import frc.robot.commands.RumbleControllerWhenDriving;
import frc.robot.commands.StartInTake;
import frc.robot.commands.StopInTake;
import frc.robot.subsystems.*;

import java.util.ArrayList;
import java.util.List;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final CommandXboxController driver = new CommandXboxController(0);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver.getHID(), XboxController.Button.kY.value);
    private final JoystickButton robotCentric = new JoystickButton(driver.getHID(), XboxController.Button.kLeftBumper.value);

    private final Trigger cancelRotateToAngle = new Trigger(() -> (driver.getRightX() > 0.1 || driver.getRightX() < -0.1));

    /* Subsystems */
    private final SwerveDrive swerveDrive = new SwerveDrive();
    private final SubIntake intake = new SubIntake();

    // Sendable Choosers
    private final SendableChooser<Command> autoChooser;
    private final SendableChooser<Integer> numOfAutoActions;
    private List<SendableChooser<Command>> selectedPathActions = new ArrayList<>();
    private List<SendableChooser<Command>> selectedNoteActions = new ArrayList<>();
    private boolean hasSetupAutoChoosers = false;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        swerveDrive.setDefaultCommand(
                swerveDrive.run(() -> swerveDrive.teleopDriveSwerve(
                        driver::getLeftY,
                        driver::getLeftX,
                        driver::getRightX,
                        () -> driver.leftBumper().getAsBoolean()
                ))
        );

        // Configure the button bindings
        configureButtonBindings();

        // PathPlanner command templates
        NamedCommands.registerCommand("marker1", Commands.print("Passed marker 1"));
        NamedCommands.registerCommand("marker2", Commands.print("Passed marker 2"));
        NamedCommands.registerCommand("print hello", Commands.print("Hello"));
        NamedCommands.registerCommand("EndPathAction", Commands.print("End of the Path Action"));
        NamedCommands.registerCommand("EndNoteAction", Commands.print("End of the Note Action"));

        // Chooser for number of actions in auto
        numOfAutoActions = new SendableChooser<>();
        numOfAutoActions.setDefaultOption("0", 0);
        for (int i = 1; i <= 5; i++) {
            numOfAutoActions.addOption("" + i, i);
        }
        SmartDashboard.putData("Number Of Auto Actions", numOfAutoActions);
        // Auto Chooser
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Mode", autoChooser);
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * Joystick} or {@link XboxController}), and then passing it to a {@link
     * JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(swerveDrive::zeroHeading));
        driver.povCenter().onFalse(swerveDrive.run(() -> swerveDrive.teleopDriveSwerve(
                () -> driver.getRawAxis(translationAxis),
                () -> driver.getRawAxis(strafeAxis),
                () -> swerveDrive.rotationPercentageFromTargetAngle(Rotation2d.fromDegrees(driver.getHID().getPOV())),
                robotCentric
                )));
        cancelRotateToAngle.onTrue(new InstantCommand(swerveDrive::cancelCurrentCommand));

        driver.leftStick().toggleOnTrue(new RumbleControllerWhenDriving(driver));

        driver.rightStick().toggleOnTrue(swerveDrive.run(() -> swerveDrive.teleopDriveSwerve(
                () -> driver.getRawAxis(translationAxis),
                () -> driver.getRawAxis(strafeAxis),
                () -> swerveDrive.rotationPercentageFromTargetAngle(swerveDrive.getAngleToPose(
                        new Translation2d(Units.inchesToMeters(-1.5), Units.inchesToMeters(218.42)))),
                robotCentric
        )));

        driver.a().onTrue(new StartInTake(intake));
        driver.b().onTrue(new StopInTake(intake));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        //return autoChooser.getSelected();

        Command[] autoCommands = new Command[numOfAutoActions.getSelected()*2];

        for (int i = 0; i < (autoCommands.length/2); i++) {
            autoCommands[(i*2)] = selectedPathActions.get(i).getSelected();
            autoCommands[(i*2)+1] = selectedNoteActions.get(i).getSelected();
        }

        return new SequentialCommandGroup(autoCommands);
    }

    /**
     * Creates all the {@link SendableChooser} for Autonomous
     */
    public void setupAutoChoosers() {
        if(!hasSetupAutoChoosers) {
            for (int i = 0; i < 1; i++) {
                // Sendable Choosers from Custom Pathplanner AutoBuilder
                SendableChooser<Command> pathAction = CustomAutoBuilder.buildAutoChooserFromAutosInPPFolder("Path Actions");
                SendableChooser<Command> noteAction = CustomAutoBuilder.buildAutoChooserFromAutosInPPFolder("Note Actions");


                // Sendable Choosers for testing purposes only
                selectedPathActions.add(pathAction);
                selectedNoteActions.add(noteAction);

//            // Set Default selections
//            selectedPathActions.get(i).setDefaultOption("Path Action 1", Commands.print("Command Path Action 1"));
//            selectedNoteActions.get(i).setDefaultOption("Note Action 1", Commands.print("Command Note Action 1"));
//
//            for (int j = 2; j <= 5; j++) {
//                selectedPathActions.get(i).addOption("Path Action " + j, Commands.print("Command Path Action " + j));
//                selectedNoteActions.get(i).addOption("Note Action " + j, Commands.print("Command Note Action " + j));
//            }

                // Send Choosers to the dashboard
                SmartDashboard.putData("Path Action " + (i + 1), selectedPathActions.get(i));
                SmartDashboard.putData("Note Action " + (i + 1), selectedNoteActions.get(i));
            }
            hasSetupAutoChoosers = true;
        }
    }
}
