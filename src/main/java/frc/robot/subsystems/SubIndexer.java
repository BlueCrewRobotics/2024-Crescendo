// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class SubIndexer extends SubsystemBase {
  /** Creates a new SubIndexer. */

  private CANSparkMax indexerMotor = new CANSparkMax(Constants.INDEXER_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless);
  private SparkPIDController indexerController = indexerMotor.getPIDController();
  private SparkRelativeEncoder indexerEncoder = (SparkRelativeEncoder) indexerMotor.getEncoder();
  private double IndexerMaxLimit = Constants.INDEXER_MAX_LIMIT;
  private double IndexerMinLimit = Constants.INDEXER_MIN_LIMIT;

  public SubIndexer() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void configureMotor(){
    indexerMotor.setSmartCurrentLimit(20);
    indexerMotor.setSoftLimit(CANSparkBase.SoftLimitDirection.kForward,(float) IndexerMaxLimit);
    indexerMotor.setSoftLimit(CANSparkBase.SoftLimitDirection.kReverse, (float) IndexerMinLimit);

  }


}
