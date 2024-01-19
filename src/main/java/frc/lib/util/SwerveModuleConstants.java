package frc.lib.util;

import edu.wpi.first.math.geometry.Rotation2d;

public class SwerveModuleConstants {
    public final int driveMotorID;
    public final int angleMotorID;
    public final int cancoderID;
    public final Rotation2d angleOffset;

    /**
     * Swerve Module Constants to be used when creating swerve modules.
     * @param driveMotorID CAN Bus ID of the drive motor
     * @param angleMotorID CAN Bus ID of the angle motor
     * @param CANcoderID Can Bus ID of the canCoder
     * @param angleOffset The wheels angle offset from facing forwards
     */
    public SwerveModuleConstants(int driveMotorID, int angleMotorID, int CANcoderID, Rotation2d angleOffset) {
        this.driveMotorID = driveMotorID;
        this.angleMotorID = angleMotorID;
        this.cancoderID = CANcoderID;
        this.angleOffset = angleOffset;
    }
}
