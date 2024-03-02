package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.*;


import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.bluecrew.util.BlinkinValues;
import frc.lib.bluecrew.util.FieldState;
import frc.lib.bluecrew.util.RobotState;
import frc.robot.autos.AutonomousCommandsBuilder;
import frc.robot.subsystems.*;
import frc.robot.subsystems.noteplayer.NotePlayerSubsystem;
import frc.robot.subsystems.swervedrive.SwerveDrive;

import java.util.Objects;
import java.util.function.BooleanSupplier;

import frc.robot.commands.*;

import static frc.robot.Constants.NotePlayerConstants.ARM_UNDER_STAGE_ANGLE_THRESHOLD;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer implements Constants.AutoConstants {
    /* Controllers */
    private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandXboxController auxDriver = new CommandXboxController(1);

    /* Driver Buttons */
    private final BooleanSupplier cancelAutoRotation = () -> (driver.getRightX() > 0.1 || driver.getRightX() < -0.1);
    private final Trigger intakeNote = new Trigger(driver.getHID()::getRightBumper);

    /* Subsystems */
    private final SwerveDrive swerveDrive = new SwerveDrive();
    private final NotePlayerSubsystem notePlayerSubsystem = new NotePlayerSubsystem();

    private final ClimberSubsystem climberSubsystem = ClimberSubsystem.getInstance();

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

    private int autoLastNumOfNotes;
    private int autoLastNumOfAmps;
    private int autoLastNumOfStartNotes;
    private String autoLastAutoLane;
    private String autoLastSearchDirection;
    private boolean autoLastGrabFromCenterFirst;

    private Command autoCommand;

    private ShuffleboardTab autonomousTab = Shuffleboard.getTab("Autonomous");

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {

        // Configure the button bindings
        configureButtonBindings();

        // PathPlanner command templates
        NamedCommands.registerCommand("marker1", Commands.print("Passed marker 1"));
        NamedCommands.registerCommand("marker2", Commands.print("Passed marker 2"));
        NamedCommands.registerCommand("print hello", Commands.print("Hello"));
        NamedCommands.registerCommand("EndPathAction", Commands.print("End of the Path Action"));
        NamedCommands.registerCommand("EndNoteAction", Commands.print("End of the Note Action"));

        NamedCommands.registerCommand("IntakeNote", notePlayerSubsystem.intakeNote());

/*
        Thread visionThread = new Thread(new VisionPipelineRunnable(VisionModule.getInstance()), "visionThread");
        visionThread.setDaemon(true);
        visionThread.start();
*/
        setupAutoChoosers();

        // Fire up the blinkin
        BlinkinSubsystem.getInstance().setColorMode(BlinkinValues.BLUE);

        autoLastNumOfNotes = numOfNotesToScoreChooser.getSelected();
        autoLastNumOfAmps = numOfAmpScoresChooser.getSelected();
        autoLastNumOfStartNotes = numOfNotesFromStartChooser.getSelected();
        autoLastAutoLane = autoLaneChooser.getSelected();
        autoLastSearchDirection = directionToSearchInChooser.getSelected();
        autoLastGrabFromCenterFirst = grabFromCenterFirstChooser.getSelected();

        autoCommand = new AutonomousCommandsBuilder(autoLastNumOfNotes, autoLastNumOfAmps,
                autoLastAutoLane, autoLastNumOfStartNotes, autoLastSearchDirection,
                autoLastGrabFromCenterFirst, notePlayerSubsystem, swerveDrive);
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * Joystick} or {@link XboxController}), and then passing it to a {@link
     * JoystickButton}.
     */
    private void configureButtonBindings() {
        // Default Commands:
        swerveDrive.setDefaultCommand(
                swerveDrive.teleopDriveSwerveDriveCommand(
                        driver::getLeftY,
                        driver::getLeftX,
                        driver::getRightTriggerAxis,
                        driver::getRightX,
                        () -> false//driver.leftBumper().getAsBoolean()
                ));

//        /* Driver Buttons */
//        driver.povCenter().onFalse(Commands.waitSeconds(0.1).andThen(swerveDrive.setHoldHeading(-driver.getHID().getPOV()).until(cancelAutoRotation)));

        driver.rightTrigger(0.75).onTrue(swerveDrive.faceSpeaker().until(cancelAutoRotation));

        driver.x().onTrue(new InstantCommand(swerveDrive::xLockWheels));

        driver.a().onTrue(swerveDrive.invertControls());

        driver.start().whileTrue(swerveDrive.alignWithAmp());

        driver.rightBumper().whileTrue(new FindAndGotoNote(swerveDrive)
                .until(notePlayerSubsystem.getIntake()::noteInIntake)
        );//.alongWith(Commands.waitUntil(RobotState.getInstance()::isNoteIsAvailable).andThen(notePlayerSubsystem.intakeNote())));
        intakeNote.whileTrue(Commands.waitUntil(RobotState.getInstance()::isNoteIsAvailable).andThen(notePlayerSubsystem.intakeNote()));
//        driver.rightBumper().onTrue(Commands.waitUntil(RobotState.getInstance()::isNoteIsAvailable).withTimeout(1)
//                .andThen(notePlayerSubsystem.intakeNote().onlyIf(RobotState.getInstance()::isNoteIsAvailable)));

        driver.leftBumper().onTrue(notePlayerSubsystem.scoreNote());

        auxDriver.leftBumper().whileTrue(notePlayerSubsystem.driveArmPercent(() -> 0.15));
        auxDriver.rightBumper().whileTrue(notePlayerSubsystem.driveArmPercent(() -> -0.125));
        auxDriver.b().whileTrue(notePlayerSubsystem.aimAndSpinUpForSpeaker());
        auxDriver.a().onTrue(notePlayerSubsystem.prepForPickup());
        auxDriver.y().onTrue(notePlayerSubsystem.prepForAmp());

        auxDriver.x().whileTrue(notePlayerSubsystem.eject());

        auxDriver.povUp().onTrue(climberSubsystem.prepForClimbCommand());
        auxDriver.povDown().onTrue(climberSubsystem.doClimbClimbCommand());

        auxDriver.rightTrigger().whileTrue(notePlayerSubsystem.intakeNote());

        auxDriver.rightStick().onTrue(notePlayerSubsystem.rotateArmToDegrees(50));

        // Robot Status Triggers

        new Trigger(() -> notePlayerSubsystem.getArm().getShooterDegrees() < ARM_UNDER_STAGE_ANGLE_THRESHOLD)
                .onTrue(new RumbleController(driver.getHID(), 0.25)
                        .andThen(Commands.waitSeconds(0.1))
                        .andThen(new RumbleController(driver.getHID(), 0.25)));

        new Trigger(notePlayerSubsystem.getIndexer()::noteInIndexer)
                .onTrue(new RumbleController(auxDriver.getHID(), GenericHID.RumbleType.kLeftRumble, 0.1)
                        .andThen(new RumbleController(auxDriver.getHID(), GenericHID.RumbleType.kRightRumble, 0.1))
                        .andThen(Commands.waitSeconds(0.1))
                        .andThen(new RumbleController(auxDriver.getHID(), GenericHID.RumbleType.kBothRumble, 0.1)));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoOptionsHaveChanged() ? new AutonomousCommandsBuilder(numOfNotesToScoreChooser.getSelected(), numOfAmpScoresChooser.getSelected(),
                autoLaneChooser.getSelected(), numOfNotesFromStartChooser.getSelected(),
                directionToSearchInChooser.getSelected(), grabFromCenterFirstChooser.getSelected(),
                notePlayerSubsystem, swerveDrive).alongWith((Commands.waitSeconds(0.25).andThen(Commands.print("PERIODIC 1/4 SECOND TIME STAMP: " + System.nanoTime()/1E9))))

                : autoCommand;

//        return new RunCommand(() -> notePlayerSubsystem.getIntake().spin(0.4)).withTimeout(4);
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
        autoLaneChooser.addOption("AmpSideLane", ampLane);
        autoLaneChooser.addOption("UnderStageLane", stageLane);
        autoLaneChooser.addOption("SourceSideLane", sourceLane);

        // Choose how many notes to get from the starting zone
        numOfNotesFromStartChooser = new SendableChooser<>();
        numOfNotesFromStartChooser.setDefaultOption("3", 3);
        numOfNotesFromStartChooser.addOption("0", 0);
        numOfNotesFromStartChooser.addOption("1", 1);
        numOfNotesFromStartChooser.addOption("2", 2);

        // Choose Which direction the robot will search for notes in
        directionToSearchInChooser = new SendableChooser<>();
        directionToSearchInChooser.setDefaultOption("FromAmp", "FromAmp");
        directionToSearchInChooser.addOption("FromSource", "FromSrc");
        directionToSearchInChooser.addOption("MiddleOut", "MidOut");

        // Choose whether to grab notes from the center or the start first
        grabFromCenterFirstChooser = new SendableChooser<>();
        grabFromCenterFirstChooser.setDefaultOption("GrabFromStartFirst", false);
        grabFromCenterFirstChooser.addOption("GrabFromCenterFirst", true);

        autonomousTab.add("Number Of Auto Actions", numOfNotesToScoreChooser).withWidget(BuiltInWidgets.kSplitButtonChooser);
        autonomousTab.add("Number Of Amp Scores", numOfAmpScoresChooser).withWidget(BuiltInWidgets.kSplitButtonChooser);
        autonomousTab.add("Autonomous Lane", autoLaneChooser).withWidget(BuiltInWidgets.kSplitButtonChooser);
        autonomousTab.add("Number Of Notes From Start", numOfNotesFromStartChooser).withWidget(BuiltInWidgets.kSplitButtonChooser);
        autonomousTab.add("Direction To Search In", directionToSearchInChooser).withWidget(BuiltInWidgets.kSplitButtonChooser);
        autonomousTab.add("Grab From Where", grabFromCenterFirstChooser).withWidget(BuiltInWidgets.kSplitButtonChooser);
    }

    public boolean autoOptionsHaveChanged() {
        int currentNumOfNotesToScore = numOfNotesToScoreChooser.getSelected();
        int currentNumOfAmps = numOfAmpScoresChooser.getSelected();
        int currentNumOfNotesFromStart = numOfNotesFromStartChooser.getSelected();
        String currentAutoLane = autoLaneChooser.getSelected();
        String currentSearchDirection = directionToSearchInChooser.getSelected();
        boolean currentGrabFromStartFirst = grabFromCenterFirstChooser.getSelected();
        if ((currentNumOfNotesToScore != autoLastNumOfNotes) || (currentNumOfAmps != autoLastNumOfAmps) ||
                (currentNumOfNotesFromStart != autoLastNumOfStartNotes) || (!Objects.equals(currentAutoLane, autoLastAutoLane)) ||
                (!Objects.equals(currentSearchDirection, autoLastSearchDirection)) || (currentGrabFromStartFirst != autoLastGrabFromCenterFirst)) {

            autoLastNumOfNotes = currentNumOfNotesToScore;
            autoLastNumOfAmps = currentNumOfAmps;
            autoLastNumOfStartNotes = currentNumOfNotesFromStart;
            autoLastAutoLane = currentAutoLane;
            autoLastSearchDirection = currentSearchDirection;
            autoLastGrabFromCenterFirst = currentGrabFromStartFirst;

            return true;
        } else {
            return false;
        }
    }

    public void regenerateAutoCommand() {
        autoCommand = new AutonomousCommandsBuilder(autoLastNumOfNotes, autoLastNumOfAmps,
                autoLastAutoLane, autoLastNumOfStartNotes, autoLastSearchDirection,
                autoLastGrabFromCenterFirst, notePlayerSubsystem, swerveDrive);
    }
}
