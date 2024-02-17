// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.noteplayer;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.*;


import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotState;
import frc.robot.Constants;

public class ArmModule implements Constants.NotePlayerConstants {

    private final CANSparkMax leftMotor;
    private final CANSparkMax rightMotor;
    private final SparkPIDController leftController;
    private final SparkRelativeEncoder leftEncoder;
    private final SparkPIDController rightController;
    private final SparkRelativeEncoder rightEncoder;

    private final double motorRotationsPerArmDegree = 0.17924558;
    private final double gravityFF = 0.065;
    private final double positionTolerance = 0.1;

    private Double setPosition = null;

    private double percentOut = 0;

    private double pseudoBottomLimit = -10;
    private double pseudoTopLimit = 5;

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
        //System.out.println("Setting Arm Position: " + (-degrees*motorRotationsPerArmDegree));
        return -degrees * motorRotationsPerArmDegree;
    }

    public void resetMotorEncoderToAbsolute() {
        double newPosition = getArmDegrees() * motorRotationsPerArmDegree;

        leftEncoder.setPosition(newPosition);
    }

    public void rotateToDegrees(double degrees) {
        setPosition = armDegreesToMotorRotations(degrees);
    }

    public double getArmDegrees() {
        return -armCANcoder.getAbsolutePosition().getValue() * 360;
    }

    public double getShooterDegrees() {
        return armCANcoder.getAbsolutePosition().getValue() * 360;
    }

    public void rotatePercentOut(double percentOut) {
        this.percentOut = percentOut;
    }

    public void setPseudoLimits(double position) {
        System.out.println("Setting Arm Position: " + position);
        double currentPosition = leftEncoder.getPosition();
        if (currentPosition > position) {
            pseudoTopLimit = currentPosition;
            pseudoBottomLimit = position;
        } else {
            pseudoBottomLimit = currentPosition;
            pseudoTopLimit = position;
        }
        setPosition = position;
    }

    /**
     * @return True if the position of the arm is within {@value ARM_POSITION_ERROR_TOLERANCE} degrees of the setpoint
     */
    public boolean isAtSetPosition() {
        //System.out.println("Arm Lower Bound Tolerance: " + (setPosition-(ARM_POSITION_ERROR_TOLERANCE * motorRotationsPerArmDegree)) + ", Upper Bound: " + (setPosition+(ARM_POSITION_ERROR_TOLERANCE * motorRotationsPerArmDegree)));
        return getArmDegrees() * motorRotationsPerArmDegree >= setPosition - (ARM_POSITION_ERROR_TOLERANCE * motorRotationsPerArmDegree)
                && getArmDegrees() * motorRotationsPerArmDegree <= setPosition + (ARM_POSITION_ERROR_TOLERANCE * motorRotationsPerArmDegree);
    }

    private void configureMotors() {


        double p = 0.2;
        double i = 0.0;
        double d = 0.3;
        double iZ = 0.0;
        // Output range
        double maxOutput = 0.48;
        double minOutput = -0.48;

        leftMotor.restoreFactoryDefaults();
        rightMotor.restoreFactoryDefaults();

        leftController.setP(p);
        leftController.setI(i);
        leftController.setD(d);
        leftController.setIZone(iZ);
        leftController.setOutputRange(minOutput, maxOutput);

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
//        if (setPosition != null) { // TODO: add && !isAtSetPosition()
//            // Calculate feed forward based on angle to counteract gravity
//            double sineScalar = Math.sin(Units.rotationsToRadians(armCANcoder.getAbsolutePosition().getValue()) - ARM_BALANCE_DEGREES);
//            double feedForward = gravityFF * sineScalar;
//            leftController.setReference(setPosition,
//                    CANSparkBase.ControlType.kPosition, 0, feedForward, SparkPIDController.ArbFFUnits.kPercentOut);
//        }

        // Calculate feed forward based on angle to counteract gravity
        double sineScalar = Math.sin(Units.rotationsToRadians(armCANcoder.getAbsolutePosition().getValue()) - ARM_BALANCE_DEGREES);
        double feedForward = gravityFF * sineScalar;
        if ((percentOut < 0 && leftEncoder.getPosition() > pseudoBottomLimit) || (percentOut > 0 && leftEncoder.getPosition() < pseudoTopLimit)) {
            leftMotor.set(percentOut + feedForward);
        } else {
            leftMotor.set(feedForward);
        }
        if (RobotState.isDisabled()) {
            resetMotorEncoderToAbsolute();
            setPosition = getArmDegrees() * motorRotationsPerArmDegree;
        }
        //System.out.println("Shooter Angle: " + getShooterDegrees());
//        if (isAtSetPosition()) {
//            System.out.println("Arm at set position!");
//        } else {
//            System.out.println("Arm set position: " + setPosition + ", Actual: " + leftEncoder.getPosition() + ", Arm Degrees: " + getArmDegrees() * motorRotationsPerArmDegree);
//        }
    }
}
