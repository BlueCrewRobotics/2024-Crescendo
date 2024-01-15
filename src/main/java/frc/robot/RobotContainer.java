package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;


import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.RumbleControllerWhenDriving;
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

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();

    // Sendable Choosers
    private final SendableChooser<Command> autoChooser;
    private final SendableChooser<Integer> numOfAutoActions;
    private List<SendableChooser<Command>> sellectedPathActions = new ArrayList<>();
    private List<SendableChooser<Command>> sellectedNoteActions = new ArrayList<>();
    private boolean hasSetupChoosers = false;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
                s_Swerve.run(() -> s_Swerve.teleopDriveSwerve(
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
        zeroGyro.onTrue(new InstantCommand(s_Swerve::zeroHeading));
        driver.povCenter().onFalse(s_Swerve.teleopDriveSwerveAndRotateToDPadCommand(
                () -> driver.getRawAxis(translationAxis),
                () -> driver.getRawAxis(strafeAxis),
                () -> driver.getRawAxis(rotationAxis),
                robotCentric
        ));
        Trigger cancelRotateToAngle = new Trigger(() -> (driver.getRightX() > 0.1 || driver.getRightX() < -0.1));
        cancelRotateToAngle.onTrue(new InstantCommand(s_Swerve::cancelRotateToDPad));

        driver.rightStick().toggleOnTrue(new RumbleControllerWhenDriving(driver));
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
            autoCommands[(i*2)] = sellectedPathActions.get(i).getSelected();
            autoCommands[(i*2)+1] = sellectedNoteActions.get(i).getSelected();
        }

        return new SequentialCommandGroup(autoCommands);
    }

    public void setupNumOfAutoActions() {
        if(!hasSetupChoosers) {
            for (int i = 0; i < 5; i++) {
                // Sendable Choosers from Custom Pathplanner AutoBuilder
//                SendableChooser<Command> pathAction = CustomAutoBuilder.buildAutoChooserFromAutosInPPFolder("Path Actions");
//                SendableChooser<Command> noteAction = CustomAutoBuilder.buildAutoChooserFromAutosInPPFolder("Note Actions");


                // Sendable Choosers for testing purposes only
                sellectedPathActions.add(new SendableChooser<>());
                sellectedNoteActions.add(new SendableChooser<>());

                // Set Default selections
                sellectedPathActions.get(i).setDefaultOption("Path Action 1", Commands.print("Command Path Action 1"));
                sellectedNoteActions.get(i).setDefaultOption("Note Action 1", Commands.print("Command Note Action 1"));

                for (int j = 2; j <= 5; j++) {
                    sellectedPathActions.get(i).addOption("Path Action " + j, Commands.print("Command Path Action " + j));
                    sellectedNoteActions.get(i).addOption("Note Action " + j, Commands.print("Command Note Action " + j));
                }

                // Send Choosers to the dashboard
                SmartDashboard.putData("Path Actions " + (i+1), sellectedPathActions.get(i));
                SmartDashboard.putData("Note Actions " + (i+1), sellectedNoteActions.get(i));
            }
            hasSetupChoosers = true;
        }
    }
}
