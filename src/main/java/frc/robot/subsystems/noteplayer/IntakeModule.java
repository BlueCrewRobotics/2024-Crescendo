// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.noteplayer;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

public class IntakeModule implements Constants.NotePlayerConstants {

    private TalonFX intakeMotor = new TalonFX(INTAKE_MOTOR_ID);

    private final NotePlayerCTREConfigs ctreConfigs = new NotePlayerCTREConfigs();
    
    private final DigitalInput beamBreak;

    public IntakeModule() {

        intakeMotor.getConfigurator().apply(ctreConfigs.intakeConfig);

        intakeMotor.setSafetyEnabled(false);

        beamBreak = new DigitalInput(8);
    }

    public void spin(double speed) {
        intakeMotor.setControl(new DutyCycleOut(speed));
    }

    public void stop() {
        intakeMotor.setControl(new DutyCycleOut(0));
    }

    public double getIntakeVelocity() {
        return intakeMotor.getVelocity().getValue();
    }

    /**
     * @return whether or not a note can be seen in the intake
     */
    public boolean noteInIntake() {
        boolean notePresent = !beamBreak.get();
        return notePresent;
    }
}
