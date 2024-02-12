// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.noteplayer;

import com.revrobotics.*;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;


public class IndexerModule {

  private CANSparkMax indexerMotor = new CANSparkMax(Constants.INDEXER_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless);
  private SparkPIDController indexerController = indexerMotor.getPIDController();
  private SparkRelativeEncoder indexerEncoder = (SparkRelativeEncoder) indexerMotor.getEncoder();

  private DigitalInput beamBreak = new DigitalInput(9);

  public IndexerModule() {
    indexerMotor.setSmartCurrentLimit(20);
  }

  public void spin(double speed) {
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

  // TODO: Set GlobalVariables.hasNote using noteInIndexer
}
