package frc.lib.bluecrew.util;


import frc.robot.Constants;

import java.util.Arrays;

/**
 * This class is singleton, and contains all the global variables that we use throughout a match.
 */
public final class RobotState implements Constants.GameStateConstants {

    private static RobotState instance;

    private int numOfAutoActionsToDo;
    private int numOfAutoActionsAttempted;

    // Probably actually just want two booleans (one for if we have a note, one for if we can score in the speaker)
    private RobotCycleStatus robotCycleStatus;
    private boolean hasNote;

    private boolean hasSpeakerTarget;

    private boolean autoPieceIsAvailable;

    private RobotState() {
        autoPieceIsAvailable = false;
        hasNote = true;
    }

    public static synchronized RobotState getInstance() {
        if (instance == null) {
            instance = new RobotState();
        }
        return instance;
    }

    /**
     * Should only be used at the start of autonomous or similar
     */
    public void resetVariables() {
        instance = new RobotState();
    }

    public void setRobotCycleStatus(RobotCycleStatus status) {
        robotCycleStatus = status;
    }

    public RobotCycleStatus getRobotCycleStatus() {
        return robotCycleStatus;
    }

    public void setAutoPieceIsAvailable(boolean autoPieceIsAvailable) {
        System.out.println("Setting Auto Piece Availability: " + autoPieceIsAvailable);
        this.autoPieceIsAvailable = autoPieceIsAvailable;
    }

    public boolean isAutoPieceIsAvailable() {
        System.out.println("Getting Auto Piece Availability: " + autoPieceIsAvailable);
        return autoPieceIsAvailable;
    }

    public boolean hasNote() {
        return hasNote;
    }

    public void setHasNote(boolean hasNote) {
        this.hasNote = hasNote;
    }

    public boolean hasSpeakerTarget() {
        return hasSpeakerTarget;
    }

    public void setHasSpeakerTarget(boolean hasSpeakerTarget) {
        this.hasSpeakerTarget = hasSpeakerTarget;
    }
}
