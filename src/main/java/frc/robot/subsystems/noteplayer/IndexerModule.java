// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.noteplayer;

import com.revrobotics.*;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;


public class IndexerModule implements Constants.NotePlayerConstants {

    private final CANSparkMax indexerMotor = new CANSparkMax(INDEXER_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless);
//    private final SparkLimitSwitch limitSwitch = indexerMotor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);

    private final DigitalInput beamBreak = new DigitalInput(9);

    private boolean limitSwitchState;

    public IndexerModule() {
        indexerMotor.setSmartCurrentLimit(20);
        indexerMotor.setIdleMode(CANSparkBase.IdleMode.kCoast);
        indexerMotor.enableVoltageCompensation(12);
//        limitSwitch.enableLimitSwitch(false);
        limitSwitchState = false;
    }

    public void spin(double speed) {
//        System.out.println("Spinning Indexer: " + speed);
        indexerMotor.set(speed);
    }

    public void stop() {
        indexerMotor.stopMotor();
    }

    public double getEncoderPosition() {
        return indexerMotor.getEncoder().getPosition();
    }

    /**
     * @return whether or not we "see" a note in the indexer (via beam-break)
     */
    public boolean noteInIndexer() {
        return !beamBreak.get();
    }

    public void setEnableHardLimit(boolean enableHardLimit) {
        if (limitSwitchState != enableHardLimit) {
//            limitSwitch.enableLimitSwitch(enableHardLimit);
            limitSwitchState = enableHardLimit;
        }
    }

    public boolean isLimitSwitchEnabled() {
        return limitSwitchState;
    }
}
