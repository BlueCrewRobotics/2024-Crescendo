package frc.lib.bluecrew.util;


import java.util.Arrays;
import java.util.concurrent.atomic.AtomicInteger;

import static frc.robot.Constants.GameStateConstants.*;

public final class GlobalVariables {

    private static GlobalVariables instance;

    private int numOfAutoActionsToDo;
    private int numOfAutoActionsAttempted;
    private boolean[] centerNotesExist;
    private int centerNoteIndex;

    private RobotCycleStatus robotCycleStatus;

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
        centerNoteIndex = 0;
    }

    public static synchronized GlobalVariables getInstance() {
        if(instance == null) {
            instance = new GlobalVariables();
        }
        return instance;
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
}
