package frc.lib.bluecrew.util;

import frc.robot.Constants;

import java.util.Arrays;

public class FieldState implements Constants.GameStateConstants {

    private static FieldState instance;

    private boolean[] centerNotesExist;
    private int centerNoteIndex;
    private boolean centerNotesGone;

    private FieldState() {
        centerNotesExist = new boolean[]{
                true,
                true,
                true,
                true,
                true
        };

        centerNotesGone = false;
        centerNoteIndex = 0;
    }

    public static synchronized FieldState getInstance() {
        if (instance == null) {
            instance = new FieldState();
        }
        return instance;
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
        if (index < centerNotesExist.length) centerNotesExist[index] = val;
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

}