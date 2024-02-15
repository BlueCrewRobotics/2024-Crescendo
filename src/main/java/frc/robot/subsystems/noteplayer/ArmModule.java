// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.noteplayer;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.*;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.RobotState;
import frc.robot.Constants;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;


public class ArmModule implements Constants.NotePlayerConstants {

    private final CANSparkMax leftMotor;
    private final CANSparkMax rightMotor;
    private final SparkPIDController leftController;
    private final SparkRelativeEncoder leftEncoder;
    private final SparkPIDController rightController;
    private final SparkRelativeEncoder rightEncoder;

    private final ArmFeedforward armFeedforward;
    private final ProfiledPIDController armPID;

    private final double motorRotationsPerArmDegree = 0.17924558;

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

        // Feedforward values
        double kS = 0.0;
        double kG = 0.0;
        double kV = 0.0;
        double kA = 0.0;

        // PID values
        double kP = 0.0;
        double kI = 0.0;
        double kD = 0.0;

        // Constraints, Maximum velocity and acceleration
        double maxVel = 1;
        double maxAcc = 0.5;

        armFeedforward = new ArmFeedforward(kS, kG, kV, kA);

        armPID = new ProfiledPIDController(kP, kI, kD, new TrapezoidProfile.Constraints(maxVel, maxAcc), 0.02);
    }

    public double armDegreesToMotorRotations(double degrees) {
        return -degrees * motorRotationsPerArmDegree;
    }

    public void resetMotorEncoderToAbsolute() {
        double newPosition = getCANcoderDegrees() * motorRotationsPerArmDegree;

        leftEncoder.setPosition(newPosition);
    }

    public void rotateToDegrees(double degrees) {
        armPID.setGoal(Units.degreesToRadians(degrees));
    }

    public void rotateToRadians(double radians) {
        armPID.setGoal(radians);
    }

    public double getCANcoderDegrees() {
        return -armCANcoder.getAbsolutePosition().getValue() * 360;
    }

    public double getCANcoderRadians() {
        return Units.rotationsToRadians(armCANcoder.getAbsolutePosition().getValue());
    }

    public void driveVolts(Measure<Voltage> volts) {
        leftMotor.setVoltage(volts.magnitude());
    }

    public Measure<Voltage> getLeftMotorVolts() {
        return Volts.of(leftMotor.getAppliedOutput() * leftMotor.getBusVoltage());
    }

    public Measure<Voltage> getRightMotorVolts() {
        return Volts.of(rightMotor.getAppliedOutput() * rightMotor.getBusVoltage());
    }

    public Measure<Angle> getSpeedRadians() {
        return Radians.of(Units.rotationsToRadians(armCANcoder.getVelocity().getValue()));
    }

    public Measure<Angle> getPositionRadians() {
        return Radians.of(Units.rotationsToRadians(armCANcoder.getAbsolutePosition().getValue()));
    }

    private void configureMotors() {

        // PID Values (P, I, D IZone
        double p = 0.6;
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
            leftMotor.setVoltage(armPID.calculate(getCANcoderRadians()) + armFeedforward.calculate(armPID.getSetpoint().position, armPID.getSetpoint().velocity));
        }

        if (RobotState.isDisabled()) {
            setPosition = -getCANcoderDegrees();
            resetMotorEncoderToAbsolute();
        }

        //System.out.println("Setpoint Degrees: " + setPosition + ", Setpoint motor rotations: " + armDegreesToMotorRotations(setPosition) + ", Current Position: " + leftMotor.getEncoder().getPosition());
    }
}
