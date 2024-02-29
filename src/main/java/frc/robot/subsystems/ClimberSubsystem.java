package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
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

    private PositionVoltage climberPositionVoltage = new PositionVoltage(0);

    private final DutyCycleOut shooterDutyCycle = new DutyCycleOut(0);

    private static final ClimberSubsystem climberInstance = new ClimberSubsystem();

    private TalonFXConfiguration climberConfig = new TalonFXConfiguration();

    private ClimberSubsystem() {
        motor1.clearStickyFaults();
        motor2.clearStickyFaults();

        /*
          .withMotorOutput(new MotorOutputConfigs()
                  .withInverted(InvertedValue.Clockwise_Positive))
          .withCurrentLimits(new CurrentLimitsConfigs()
                  .withSupplyCurrentLimit(15)
                  .withStatorCurrentLimitEnable(true)
                  .withSupplyCurrentThreshold(10));
*/

        climberConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        climberConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        climberConfig.CurrentLimits.SupplyCurrentLimit = 40;
        climberConfig.CurrentLimits.SupplyCurrentThreshold = 60;
        climberConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        climberConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ELEVATOR_MOTOR_UPPER_LIMIT_POS;
        climberConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ELEVATOR_MOTOR_LOWER_LIMIT_POS;
        climberConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        climberConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        climberConfig.Slot0.kP = 0.3;
        climberConfig.Slot0.kI = 0.005;
        climberConfig.Slot0.kD = 0.0;
        climberConfig.Slot0.kG = 0.3;

        climberConfig.Slot1.kP = 0.3;
        climberConfig.Slot1.kI = 0.005;
        climberConfig.Slot1.kD = 0.0;
        climberConfig.Slot1.kG = 0.25;

        motor1.getConfigurator().apply(climberConfig);
        motor2.getConfigurator().apply(climberConfig);
        motor2.setControl(new Follower(ELEVATOR_MOTOR_1_ID, false));
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

    public void prepForClimb() {

        motor1.setControl(climberPositionVoltage.withPosition(ELEVATOR_MOTOR_UPPER_LIMIT_POS));
//        motor1.setPosition(ELEVATOR_MOTOR_UPPER_LIMIT_POS);
    }

    public void doClimb() {
        motor1.setControl(climberPositionVoltage.withPosition(ELEVATOR_MOTOR_LOWER_LIMIT_POS));
//        motor1.setPosition(ELEVATOR_MOTOR_LOWER_LIMIT_POS);
    }

    private void runElevator(double speed) {

        climberVelocity = new VelocityVoltage(ELEVATOR_MAX_ROTATIONS_PER_SECOND * speed);

        climberVelocity.FeedForward = climberFeedForward.calculate(climberVelocity.Velocity);
        //shooterVelocity.withAcceleration(25);

        System.out.println(("Climber velocity set to: " + climberVelocity.Velocity));
        motor1.setControl(climberVelocity);
    }

    public void stopElevator() {
        motor1.stopMotor();
        System.out.println("motor1 pos: " + getClimberEncoderPos());
    }


    public Command prepForClimbCommand() {
        return this.run(
                this::prepForClimb);
    }

    public Command doClimbClimbCommand() {
        return this.run(
                this::doClimb);
    }

    public Command runClimberUpCommand() {

        return this.run(
                () -> runElevatorUp(0.03)).finallyDo(this::stopElevator);
    }


    public Command runClimberDownCommand() {

        return this.run(
                () -> runElevatorDown(-0.03)).finallyDo(this::stopElevator);
    }

    public Command stopClimberCommand() {

        return this.run(
                this::stopElevator);

    }

}
