package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.motorcontrol.Spark;

import frc.robot.Constants;

import frc.lib.bluecrew.util.GlobalVariables;

public class BlinkinSubsystem extends SubsystemBase {

    private final Spark blinkinOutput;

    public static final double BLINKIN_BREATH_BLUE = -0.15;
    public static final double BLINKIN_BREATH_RED = -0.17;
    public static final double BLINKIN_SOLID_RED = 0.61;
    public static final double BLINKIN_SOLID_ORANGE = 0.65;
    public static final double BLINKIN_SOLID_BLUE = 0.87;
    public static final double BLINKIN_SOLID_YELLOW = 0.69;
    public static final double BLINKIN_STROBE_RED = -0.11;
    public static final double BLINKIN_STROBE_BLUE = -0.09;
    public static final double BLINKIN_CONFETTI = -0.87;
    public static final double BLINKIN_BLUE_CHASE =  -0.29;
    public static final double BLINKIN_RAINBOW = -0.99;
    public static final double BLINKIN_FIRE_LARGE = -0.57;
    public static final double BLINKIN_COLOR_WAVE_FOREST = -0.37;
    public static final double BLINKIN_SOLID_BLACK = 0.99;
    public static final double BLINKIN_SOLID_PINK = 0.57;
    public static final double BLINKIN_SOLID_GREEN = 0.77;
    public static final double BLINKIN_SOLID_GOLD = 0.67;
    public static final double BLINKIN_SOLID_VIOLET = 0.91;

    GlobalVariables gv = GlobalVariables.getInstance();


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

        blinkinOutput = new Spark(Constants.BLINKIN_PORT);
    }

    @Override
    public void periodic() {
        if(gv.hasNote()  && gv.hasSpeakerTarget())
            setColorMode(BLINKIN_SOLID_GREEN);
        else if(gv.hasNote()) {
            setColorMode(BLINKIN_SOLID_ORANGE);
        }
        else {
            setColorMode(BLINKIN_BREATH_BLUE);
        }
    }

    public void setColorMode(double mode) {
        blinkinOutput.set(mode);
    }


}

