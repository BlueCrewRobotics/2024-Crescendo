// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SubIntake extends SubsystemBase {
  /** Creates a new SubIntake. */
private TalonFX intakeMotor = new TalonFX(Constants.INTAKE_MOTOR_ID);

  public SubIntake() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void spin(double speed){intakeMotor.setControl(new DutyCycleOut(speed));}

  public void stopSpinning(){intakeMotor.setControl(new DutyCycleOut(0));}

  public double getIntakeVelocity(){return intakeMotor.getVelocity().getValue();}


}
