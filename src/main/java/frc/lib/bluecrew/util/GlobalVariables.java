package frc.lib.bluecrew.util;


import java.util.Arrays;

import static frc.robot.Constants.GameStateConstants.*;

/**
 * This class is singleton, and contains all the global variables that we use throughout a match.
 */
public final class GlobalVariables {

    private static GlobalVariables instance;

    private int numOfAutoActionsToDo;
    private int numOfAutoActionsAttempted;
    private boolean[] centerNotesExist;
    private int centerNoteIndex;
    private boolean centerNotesGone;

    // Probably actually just want two booleans (one for if we have a note, one for if we can score in the speaker)
    private RobotCycleStatus robotCycleStatus;
    private boolean hasNote;

    private boolean autoPieceIsAvailable;

    private GlobalVariables() {
        centerNotesExist = new boolean[]{
                true,
                true,
                true,
                true,
                true
        };

        autoPieceIsAvailable = false;
        centerNotesGone = false;
        centerNoteIndex = 0;
        hasNote = true;
    }

    public static synchronized GlobalVariables getInstance() {
        if(instance == null) {
            instance = new GlobalVariables();
        }
        return instance;
    }

    /**
     * Should only be used at the start of autonomous or similar
     */
    public void resetVariables() {
        instance = new GlobalVariables();
    }

    public void setRobotCycleStatus(RobotCycleStatus status) {
        robotCycleStatus = status;
    }

    public RobotCycleStatus getRobotCycleStatus() {
        return robotCycleStatus;
    }

    public boolean[] getCenterNotesExist() {
        System.out.println("Getting Center Notes Exist: " + Arrays.toString(centerNotesExist));
        return centerNotesExist;
    }

    public void setCenterNotesExist(boolean[] centerNotesExist) {
        System.out.println("Setting Center Notes Exist: " + Arrays.toString(centerNotesExist));
        this.centerNotesExist = centerNotesExist;
    }

    public void setCenterNoteExists(int index, boolean val) {
        System.out.println("Setting Center Note Exists: " + index + " to: " + val);
        if(index < centerNotesExist.length) centerNotesExist[index] = val;
    }

    public void setAutoPieceIsAvailable(boolean autoPieceIsAvailable) {
        System.out.println("Setting Auto Piece Availability: " + autoPieceIsAvailable);
        this.autoPieceIsAvailable = autoPieceIsAvailable;
    }

    public boolean isAutoPieceIsAvailable() {
        System.out.println("Getting Auto Piece Availability: " + autoPieceIsAvailable);
        return autoPieceIsAvailable;
    }

    public void setCenterNoteIndex(int centerNoteIndex) {
        System.out.println("Setting Center Note Index: " + centerNoteIndex);
        this.centerNoteIndex = centerNoteIndex;
    }

    public int getCenterNoteIndex() {
        System.out.println("Getting Center Note Index: " + centerNoteIndex);
        return centerNoteIndex;
    }

    public void setCenterNotesGone(boolean centerNotesGone) {
        this.centerNotesGone = centerNotesGone;
    }

    public boolean isCenterNotesGone() {
        return centerNotesGone;
    }

    public boolean hasNote() {
        return hasNote;
    }

    public void setHasNote(boolean hasNote) {
        this.hasNote = hasNote;
    }
}
