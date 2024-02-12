// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.noteplayer;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.*;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;


public class ArmModule implements Constants.NotePlayerConstants {

    private final CANSparkMax leftMotor;
    private final CANSparkMax rightMotor;
    private final SparkPIDController leftController;
    private final SparkRelativeEncoder leftEncoder;
    private final SparkPIDController rightController;
    private final SparkRelativeEncoder rightEncoder;

    private final double motorRotationsPerArmDegree = 0.17924558;
    private final double gravityFF = 0.001;

    private Double setPosition = null;

    private final CANcoder armCANcoder;

    public ArmModule() {
        // Set up the motors:
        leftMotor = new CANSparkMax(ARM_LEFT_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless);
        rightMotor = new CANSparkMax(ARM_RIGHT_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless);

        // Set up the PID controllers
        leftController = leftMotor.getPIDController();
        rightController = leftMotor.getPIDController();

        // Set up the encoders
        leftEncoder = (SparkRelativeEncoder) leftMotor.getEncoder();
        rightEncoder = (SparkRelativeEncoder) rightMotor.getEncoder();

        // Set up the CANcoder
        armCANcoder = new CANcoder(ARM_CAN_CODER_ID);
        // Configure the motors
        configureMotors();

        // Reset the motor's integrated encoder based on the CANcoder
        resetMotorEncoderToAbsolute();
    }

    public double armDegreesToMotorRotations(double degrees) {
        return -degrees * motorRotationsPerArmDegree;
    }

    public void resetMotorEncoderToAbsolute() {
        double newPosition = getCANcoderDegrees() * motorRotationsPerArmDegree;

        leftEncoder.setPosition(newPosition);
    }

    public void rotateToDegrees(double degrees) {
        setPosition = degrees;
    }

    public double getCANcoderDegrees() {
        return -armCANcoder.getAbsolutePosition().getValue() * 360;
    }

    private void configureMotors() {

        // PID Values (P, I, D IZone
        double p = 0.65;
        double i = 0.0;
        double d = 0.25;
        double iZ = 0.0;
        // Feedforward value
        double ff = 0.0;
        // Output range
        double outPutRange = 0.35;
        // Allowed error in rotations
        double allowedErr = 0.002;

        // Smart Motion Coefficients
        double maxVel = 30; // RPM
        double maxAcc = 10;
        double minVel = 0;

        int smartMotionSlot = 0;

        leftMotor.restoreFactoryDefaults();
        rightMotor.restoreFactoryDefaults();

        leftController.setP(p);
        leftController.setI(i);
        leftController.setD(d);
        leftController.setIZone(iZ);
        leftController.setFF(ff);
        leftController.setOutputRange(-outPutRange, outPutRange);

//        leftController.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
//        leftController.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
//        leftController.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
//        leftController.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);

        // Voltage Compensation and current limits
        leftMotor.enableVoltageCompensation(12);
        leftMotor.setSmartCurrentLimit(40);
        rightMotor.setSmartCurrentLimit(40);

        // Set soft limits
        leftMotor.setSoftLimit(CANSparkBase.SoftLimitDirection.kForward, ARM_MAX_LIMIT);
        leftMotor.setSoftLimit(CANSparkBase.SoftLimitDirection.kReverse, ARM_MIN_LIMIT);
        leftMotor.enableSoftLimit(CANSparkBase.SoftLimitDirection.kForward, true);
        leftMotor.enableSoftLimit(CANSparkBase.SoftLimitDirection.kReverse, true);

        rightMotor.follow(leftMotor, true);

        // Keep the breaks on so the arm doesn't slam down when disabled
        leftMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
        rightMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);

        // Save the settings on the controllers, so they stick through power cycles
        leftMotor.burnFlash();
        rightMotor.burnFlash();
    }

    public void periodic() {
        if (setPosition != null) {
            // Calculate feed forward based on angle to counteract gravity
            double sineScalar = Math.sin(Units.rotationsToRadians(armCANcoder.getAbsolutePosition().getValue()) + Units.degreesToRadians(35));
            double feedForward = gravityFF * sineScalar;
            leftController.setReference(armDegreesToMotorRotations(setPosition),
                    CANSparkBase.ControlType.kPosition, 0, feedForward, SparkPIDController.ArbFFUnits.kPercentOut);
        }
    }
}
