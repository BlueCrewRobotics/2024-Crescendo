// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.*;
import frc.robot.Constants;


public class ArmModule {

  private CANSparkMax leftMotor = new CANSparkMax(Constants.ARM_LEFT_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless);
  private CANSparkMax rightMotor = new CANSparkMax(Constants.ARM_RIGHT_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless);
  private SparkPIDController leftController = leftMotor.getPIDController();
  private SparkRelativeEncoder leftEncoder = (SparkRelativeEncoder) leftMotor.getEncoder();
  private SparkPIDController rightController = leftMotor.getPIDController();
  private SparkRelativeEncoder rightEncoder = (SparkRelativeEncoder) rightMotor.getEncoder();
  private double maxLimit = Constants.ARM_MAX_LIMIT;
  private double minLimit = Constants.ARM_MIN_LIMIT;

  public ArmModule() {

    leftMotor.setSmartCurrentLimit(20);
    leftMotor.setSoftLimit(CANSparkBase.SoftLimitDirection.kForward, (float) maxLimit);
    leftMotor.setSoftLimit(CANSparkBase.SoftLimitDirection.kReverse, (float) minLimit);

    rightMotor.setSmartCurrentLimit(20);
    rightMotor.setSoftLimit(CANSparkBase.SoftLimitDirection.kForward, (float) maxLimit);
    rightMotor.setSoftLimit(CANSparkBase.SoftLimitDirection.kReverse, (float) minLimit);
  }
  
}
