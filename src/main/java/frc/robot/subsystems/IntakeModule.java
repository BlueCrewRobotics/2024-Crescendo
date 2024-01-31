// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeModule  {

  private TalonFX intakeMotor = new TalonFX(Constants.INTAKE_MOTOR_ID);
  // TODO: intake beam-break

  public IntakeModule() {

    TalonFXConfiguration motorFXConfig = new TalonFXConfiguration();
/*
          .withMotorOutput(new MotorOutputConfigs()
                  .withInverted(InvertedValue.Clockwise_Positive))
          .withCurrentLimits(new CurrentLimitsConfigs()
                  .withSupplyCurrentLimit(15)
                  .withStatorCurrentLimitEnable(true)
                  .withSupplyCurrentThreshold(10));
*/

    motorFXConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    intakeMotor.getConfigurator().apply(motorFXConfig);
  }

  public void spin(double speed){intakeMotor.setControl(new DutyCycleOut(speed));}

  public void stopSpinning(){intakeMotor.setControl(new DutyCycleOut(0));}

  public double getIntakeVelocity(){return intakeMotor.getVelocity().getValue();}

  /**
   * @return whether or not a note can be seen in the indexer
   */
  public boolean noteInIndexer() {

  }

}
