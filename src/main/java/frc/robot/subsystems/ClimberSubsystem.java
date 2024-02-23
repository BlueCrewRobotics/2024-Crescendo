package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/**
 *
 */
public class ClimberSubsystem extends SubsystemBase implements Constants.ElevatorConstants {

    private final TalonFX motor1 = new TalonFX(ELEVATOR_MOTOR_1_ID);
    private final TalonFX motor2 = new TalonFX(ELEVATOR_MOTOR_2_ID);

    private  VelocityVoltage climberVelocity = new VelocityVoltage(0);
    //private final SimpleMotorFeedforward climberFeedForward = new SimpleMotorFeedforward(Constants.shooterKS, Constants.shooterKV, Constants.shooterKA);

    private SimpleMotorFeedforward climberFeedForward = new SimpleMotorFeedforward(elevatorKS, elevatorKV, elevatorKA);

    private final DutyCycleOut shooterDutyCycle = new DutyCycleOut(0);

    private static final ClimberSubsystem climberInstance = new ClimberSubsystem();

    private ClimberSubsystem() {
        motor1.clearStickyFaults();
        motor2.clearStickyFaults();

        TalonFXConfiguration motor1FXConfig = new TalonFXConfiguration();

        /*
          .withMotorOutput(new MotorOutputConfigs()
                  .withInverted(InvertedValue.Clockwise_Positive))
          .withCurrentLimits(new CurrentLimitsConfigs()
                  .withSupplyCurrentLimit(15)
                  .withStatorCurrentLimitEnable(true)
                  .withSupplyCurrentThreshold(10));
*/

        motor1FXConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        SoftwareLimitSwitchConfigs limitSwitchConfigs = new SoftwareLimitSwitchConfigs()
                .withForwardSoftLimitEnable(true)
                        .withReverseSoftLimitEnable(true)
                                .withForwardSoftLimitThreshold(MOTOR_UPPER_LIMIT_POS)
                                        .withReverseSoftLimitThreshold(MOTOR_LOWER_LIMIT_POS);
        motor1FXConfig.withSoftwareLimitSwitch(limitSwitchConfigs);
        motor1FXConfig.withCurrentLimits(new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(40.0)
                .withStatorCurrentLimit(40.0)
                .withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimitEnable(true));
        motor1.setNeutralMode(NeutralModeValue.Brake);
        motor2.setNeutralMode(NeutralModeValue.Brake);

        motor1.getConfigurator().apply(motor1FXConfig);
        motor2.getConfigurator().apply(motor1FXConfig);
    }

    public static ClimberSubsystem getInstance() {
        return climberInstance;
    }

    public double getClimberVelocity() {
        return motor1.getVelocity().getValue().doubleValue();
    }

    public long getClimberEncoderPos() {
        return (long) (motor1.getPosition().getValue() * 2048.0d);
    }

    public void runElevatorUp(double speed) {
        speed = Math.abs(speed);
        runElevator(speed);
    }

    public void runElevatorDown(double speed) {
        speed = -Math.abs(speed);
        runElevator(speed);
    }

    private void runElevator(double speed) {

        climberVelocity = new VelocityVoltage(ELEVATOR_MAX_ROTATIONS_PER_SECOND * speed);

        climberVelocity.FeedForward = climberFeedForward.calculate(climberVelocity.Velocity);
        //shooterVelocity.withAcceleration(25);

        System.out.println(("Climber velocity set to: " + climberVelocity.Velocity));
        motor1.setControl(climberVelocity);
        motor2.setControl(climberVelocity);
    }

    public void stopElevator() {
        motor1.stopMotor();
        motor2.stopMotor();
        System.out.println("motor1 pos: " + getClimberEncoderPos());
    }


    public Command runClimberUpCommand() {

        return this.run(
                () -> runElevatorUp(0.03)).finallyDo(new Runnable() {
            @Override
            public void run() {
                stopElevator();
            }
        });

    }

    public Command runClimberDownCommand() {

        return this.run(
                () -> runElevatorDown(-0.03)).finallyDo(new Runnable() {
            @Override
            public void run() {
                stopElevator();
            }
        });
    }

    public Command stopClimberCommand() {

        return this.run(
                () -> stopElevator());

    }

}
