package frc.lib.bluecrew.util;


import frc.robot.Constants;

/**
 * This class is singleton, and contains all the global variables that we use throughout a match.
 */
public final class RobotState implements Constants.GameStateConstants {

    private static RobotState instance;

    private int numOfAutoActionsToDo;
    private int numOfAutoActionsAttempted;

    private RobotCycleStatus robotCycleStatus;

    private ShooterMode shooterMode;
    private ShooterStatus shooterStatus;

    private boolean noteIsAvailable;

    private boolean isAutonomous;

    private RobotState() {
        noteIsAvailable = false;
        robotCycleStatus = RobotCycleStatus.NO_NOTE_CANT_SEE_SPEAKER;
        shooterMode = ShooterMode.SPEAKER;
        shooterStatus = ShooterStatus.UNREADY;
        isAutonomous = false;
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
    public void resetRobotStates() {
        instance = new RobotState();
    }

    public void setRobotCycleStatus(RobotCycleStatus status) {
        robotCycleStatus = status;
    }

    public RobotCycleStatus getRobotCycleStatus() {
        return robotCycleStatus;
    }

    public void setShooterMode(ShooterMode shooterMode) {
        this.shooterMode = shooterMode;
    }

    public ShooterMode getShooterMode() {
        return shooterMode;
    }

    public void setShooterStatus(ShooterStatus shooterStatus) {
        this.shooterStatus = shooterStatus;
    }

    public ShooterStatus getShooterStatus() {
        return shooterStatus;
    }

    public void setNoteIsAvailable(boolean noteIsAvailable) {
        this.noteIsAvailable = noteIsAvailable;
    }

    public boolean isNoteIsAvailable() {
        return noteIsAvailable;
    }

    public boolean hasNote() {
        return switch (robotCycleStatus) {
            case HAS_NOTE_CANT_SEE_SPEAKER, HAS_NOTE_SEES_SPEAKER -> true;
            default -> false;
        };
    }

    public void setHasNote(boolean hasNote) {
        if(hasNote) {
            switch (robotCycleStatus) {
                case NO_NOTE_CANT_SEE_SPEAKER -> robotCycleStatus = RobotCycleStatus.HAS_NOTE_CANT_SEE_SPEAKER;
                case NO_NOTE_SEES_SPEAKER -> robotCycleStatus = RobotCycleStatus.HAS_NOTE_SEES_SPEAKER;
            }
        } else {
            switch (robotCycleStatus) {
                case HAS_NOTE_CANT_SEE_SPEAKER -> robotCycleStatus = RobotCycleStatus.NO_NOTE_CANT_SEE_SPEAKER;
                case HAS_NOTE_SEES_SPEAKER -> robotCycleStatus = RobotCycleStatus.NO_NOTE_SEES_SPEAKER;
            }
        }
    }

    public boolean hasSpeakerTarget() {
        return switch (robotCycleStatus) {
            case HAS_NOTE_SEES_SPEAKER, NO_NOTE_SEES_SPEAKER -> true;
            default -> false;
        };
    }

    public void setHasSpeakerTarget(boolean hasSpeakerTarget) {
        if (hasSpeakerTarget) {
            switch (robotCycleStatus) {
                case HAS_NOTE_CANT_SEE_SPEAKER -> robotCycleStatus = RobotCycleStatus.HAS_NOTE_SEES_SPEAKER;
                case NO_NOTE_CANT_SEE_SPEAKER -> robotCycleStatus = RobotCycleStatus.NO_NOTE_SEES_SPEAKER;
            }
        } else {
            switch (robotCycleStatus) {
                case HAS_NOTE_SEES_SPEAKER -> robotCycleStatus = RobotCycleStatus.HAS_NOTE_CANT_SEE_SPEAKER;
                case NO_NOTE_SEES_SPEAKER -> robotCycleStatus = RobotCycleStatus.NO_NOTE_CANT_SEE_SPEAKER;
            }
        }
    }

    public boolean isAutonomous() {
        return isAutonomous;
    }

    public void setIsAutonomous(boolean autonomous) {
        isAutonomous = autonomous;
    }
}
