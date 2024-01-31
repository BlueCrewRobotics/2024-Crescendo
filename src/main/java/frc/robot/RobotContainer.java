package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
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
import frc.robot.autos.AutonomousCommandsBuilder;
import frc.robot.commands.RumbleControllerWhenDriving;
import frc.robot.commands.StartInTake;
import frc.robot.commands.StopInTake;
import frc.robot.subsystems.*;

import java.util.function.BooleanSupplier;

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

    private final BooleanSupplier cancelAutoRotation = () -> (driver.getRightX() > 0.1 || driver.getRightX() < -0.1);

    /* Subsystems */
    private final SwerveDrive swerveDrive = new SwerveDrive();
    private final IntakeModule intake = new IntakeModule();

    // Sendable Choosers for autonomous
    // total number of notes to score (including in speaker+amp) during auto
    private SendableChooser<Integer> numOfNotesToScoreChooser;
    // number of notes to score in amp during auto
    private SendableChooser<Integer> numOfAmpScoresChooser;
    // number of notes to pick up from starting line area
    private SendableChooser<Integer> numOfNotesFromStartChooser;
    // should travel to-from center line go under stage, near amp, or near source
    private SendableChooser<String> autoLaneChooser;
    // if you travel under stage, do you go toward amp or source or go from the middle out to find notes
    private SendableChooser<String> directionToSearchInChooser;
    // prioritize getting notes from center line (over those from starting area)
    private SendableChooser<Boolean> grabFromCenterFirstChooser;


    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        swerveDrive.setDefaultCommand(
                swerveDrive.teleopDriveSwerveDriveCommand(
                        driver::getLeftY,
                        driver::getLeftX,
                        driver::getRightX,
                        () -> driver.leftBumper().getAsBoolean()
                ));

        // Configure the button bindings
        configureButtonBindings();

        // PathPlanner command templates
        NamedCommands.registerCommand("marker1", Commands.print("Passed marker 1"));
        NamedCommands.registerCommand("marker2", Commands.print("Passed marker 2"));
        NamedCommands.registerCommand("print hello", Commands.print("Hello"));
        NamedCommands.registerCommand("EndPathAction", Commands.print("End of the Path Action"));
        NamedCommands.registerCommand("EndNoteAction", Commands.print("End of the Note Action"));

        setupAutoChoosers();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * Joystick} or {@link XboxController}), and then passing it to a {@link
     * JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(swerveDrive::zeroGyro));
        driver.povCenter().onFalse(swerveDrive.teleopDriveSwerveDriveAndRotateToAngleCommand(
                () -> driver.getRawAxis(translationAxis),
                () -> driver.getRawAxis(strafeAxis),
                () -> -driver.getHID().getPOV(),
                robotCentric
                ).until(cancelAutoRotation));

        driver.leftStick().toggleOnTrue(new RumbleControllerWhenDriving(driver));

        driver.rightStick().toggleOnTrue(swerveDrive.teleopDriveSwerveDriveAndFacePosition(
                () -> driver.getRawAxis(translationAxis),
                () -> driver.getRawAxis(strafeAxis),
                new Translation2d(Units.inchesToMeters(-1.5), Units.inchesToMeters(218.42)),
                robotCentric
        ).until(cancelAutoRotation));

        driver.a().onTrue(new StartInTake(intake));
        driver.b().onTrue(new StopInTake(intake));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return new AutonomousCommandsBuilder(numOfNotesToScoreChooser.getSelected(), numOfAmpScoresChooser.getSelected(),
                autoLaneChooser.getSelected(), numOfNotesFromStartChooser.getSelected(),
                directionToSearchInChooser.getSelected(), grabFromCenterFirstChooser.getSelected());
    }

    /**
     * Creates each {@link SendableChooser} for Autonomous
     */
    public void setupAutoChoosers() {
        // Chooser for number of actions in auto
        numOfNotesToScoreChooser = new SendableChooser<>();
        numOfNotesToScoreChooser.setDefaultOption("0", 0);
        for (int i = 1; i <= 5; i++) {
            numOfNotesToScoreChooser.addOption("" + i, i);
        }

        // Choose how many notes to score in Amp
        numOfAmpScoresChooser = new SendableChooser<>();
        numOfAmpScoresChooser.setDefaultOption("0", 0);
        numOfAmpScoresChooser.addOption("1", 1);
        numOfAmpScoresChooser.addOption("2", 2);

        // Choose which lane the robot should travel in
        autoLaneChooser = new SendableChooser<>();
        autoLaneChooser.setDefaultOption("None", null);
        autoLaneChooser.addOption("AmpSideLane", Constants.AutoConstants.ampLane);
        autoLaneChooser.addOption("UnderStageLane", Constants.AutoConstants.stageLane);
        autoLaneChooser.addOption("SourceSideLane", Constants.AutoConstants.sourceLane);

        // Choose how many notes to get from the starting zone
        numOfNotesFromStartChooser = new SendableChooser<>();
        numOfNotesFromStartChooser.setDefaultOption("0", 0);
        numOfNotesFromStartChooser.addOption("1", 1);
        numOfNotesFromStartChooser.addOption("2", 2);
        numOfNotesFromStartChooser.addOption("3", 3);

        // Choose Which direction the robot will search for notes in
        // TODO: add a closest-outwards option
        directionToSearchInChooser = new SendableChooser<>();
        directionToSearchInChooser.setDefaultOption("TowardsAmp", "ToAmp");
        directionToSearchInChooser.addOption("TowardsSource", "ToSrc");
        directionToSearchInChooser.addOption("MiddleOut", "MidOut");

        // Choose whether to grab notes from the center or the start first
        grabFromCenterFirstChooser = new SendableChooser<>();
        grabFromCenterFirstChooser.setDefaultOption("GrabFromCenterFirst", true);
        grabFromCenterFirstChooser.addOption("GrabFromStartFirst", false);

        SmartDashboard.putData("Number Of Auto Actions", numOfNotesToScoreChooser);
        SmartDashboard.putData("Number Of Amp Scores", numOfAmpScoresChooser);
        SmartDashboard.putData("Autonomous Lane", autoLaneChooser);
        SmartDashboard.putData("Number Of Notes From Start", numOfNotesFromStartChooser);
        SmartDashboard.putData("Direction To Search In", directionToSearchInChooser);
        SmartDashboard.putData("Grab From Where", grabFromCenterFirstChooser);
    }
}
