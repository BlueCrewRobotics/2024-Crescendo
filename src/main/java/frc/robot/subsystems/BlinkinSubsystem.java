package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.motorcontrol.Spark;

import frc.lib.bluecrew.util.BlinkinValues;
import frc.robot.Constants;

import frc.lib.bluecrew.util.RobotState;

public class BlinkinSubsystem extends SubsystemBase  implements Constants.Misc, Constants.GameStateConstants, BlinkinValues {

    private final Spark blinkinOutput;

    RobotState gv = RobotState.getInstance();


    /**
     * The Singleton instance of this BlinkinSubsystem. Code should use
     * the {@link #getInstance()} method to get the single instance (rather
     * than trying to construct an instance of this class.)
     */
    private final static BlinkinSubsystem INSTANCE = new BlinkinSubsystem();

    /**
     * Returns the Singleton instance of this BlinkinSubsystem. This static method
     * should be used, rather than the constructor, to get the single instance
     * of this class. For example: {@code BlinkinSubsystem.getInstance();}
     */
    @SuppressWarnings("WeakerAccess")
    public static BlinkinSubsystem getInstance() {
        return INSTANCE;
    }

    /**
     * Creates a new instance of this BlinkinSubsystem. This constructor
     * is private since this class is a Singleton. Code should use
     * the {@link #getInstance()} method to get the singleton instance.
     */
    private BlinkinSubsystem() {
        blinkinOutput = new Spark(BLINKIN_PORT);
    }

    @Override
    public void periodic() {
        if(gv.hasNote()  && gv.hasSpeakerTarget())
            setColorMode(GREEN);
        else if(gv.hasNote()) {
            setColorMode(ORANGE);
        }
        else {
            setColorMode(BREATH_BLUE);
        }
    }

    public void setColorMode(double mode) {
        blinkinOutput.set(mode);
    }
}

