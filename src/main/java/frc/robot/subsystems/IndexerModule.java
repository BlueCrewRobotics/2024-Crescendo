// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class IndexerModule {

  private CANSparkMax indexerMotor = new CANSparkMax(Constants.INDEXER_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless);
  private SparkPIDController indexerController = indexerMotor.getPIDController();
  private SparkRelativeEncoder indexerEncoder = (SparkRelativeEncoder) indexerMotor.getEncoder();
  private double IndexerMaxLimit = Constants.INDEXER_MAX_LIMIT;
  private double IndexerMinLimit = Constants.INDEXER_MIN_LIMIT;

  // TODO: indexer beam-break

  public IndexerModule() {

    indexerMotor.setSmartCurrentLimit(20);
    indexerMotor.setSoftLimit(CANSparkBase.SoftLimitDirection.kForward, (float) IndexerMaxLimit);
    indexerMotor.setSoftLimit(CANSparkBase.SoftLimitDirection.kReverse, (float) IndexerMinLimit);

  }

  // TODO: Set GlobalVariables.hasNote using a beam break sensor using WPILib's DigitalInput class

  /**
   * @return whether or not we "see" a note in the intake (via beam-break)
   */
  public boolean noteInIntake() {

  }

}
