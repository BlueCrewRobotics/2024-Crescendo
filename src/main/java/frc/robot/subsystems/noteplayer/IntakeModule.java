// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.noteplayer;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeModule implements Constants.NotePlayerConstants {

    private TalonFX intakeMotor = new TalonFX(INTAKE_MOTOR_ID);

    private final NotePlayerCTREConfigs ctreConfigs = new NotePlayerCTREConfigs();
    
    private final DigitalInput beamBreak;

    public IntakeModule() {

        intakeMotor.getConfigurator().apply(ctreConfigs.intakeConfig);

        intakeMotor.setSafetyEnabled(true);

        beamBreak = new DigitalInput(8);
    }

    public void spin(double speed) {
        intakeMotor.setControl(new DutyCycleOut(speed));
    }

    public void stopSpinning() {
        intakeMotor.setControl(new DutyCycleOut(0));
    }

    public double getIntakeVelocity() {
        return intakeMotor.getVelocity().getValue();
    }

    /**
     * @return whether or not a note can be seen in the intake
     */
    public boolean noteInIntake() {
        return !beamBreak.get();
    }
}
