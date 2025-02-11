package frc.robot.subsystems;

import java.util.function.Supplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.techhounds.houndutil.houndlib.subsystems.BaseLinearMechanism;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.Elevator.Constants.*;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

/** The subsystem for the robot's elevator mechanism */
public class Elevator extends SubsystemBase implements BaseLinearMechanism<Position> {
    public static final class Constants {
        public static final class CAN{
            public static final int CANID = 0;
        }

        public static final class MotorConfigs{
            public static boolean INVERTED = false;
            public static int CURRENT_LIMIT = 0;
            public static double ENCODER_CONVERSION_FACTOR = 0.0;
        }

        public static final class Feedback{
            public static final double kP = 0.0;
            public static final double kI = 0.0;
            public static final double kD = 0.0;
            public static final double TOLERANCE = 0.05;
        }

        public static final class Feedforward{
            public static final double kG = 0.0;
            public static final double kS = 0.0;
            public static final double kV = 0.0;
            public static final double kA = 0.0;
        }

        public static final class MotionProfile{
            public static final double MAX_ACCELERATION = 0.0;
            public static final double MAX_VELOCITY = 0.0;
        }

        public static final TrapezoidProfile.Constraints ELEVATOR_PROFILE = new TrapezoidProfile.Constraints(
            MotionProfile.MAX_VELOCITY,
            MotionProfile.MAX_ACCELERATION
        );

        public static enum Position {
            RESET(0.0),
            ZERO(0.0),
            L1(0.0),
            L2(0.0),
            L3(0.0),
            L4(0.0);

            public final double position;

            private Position(final double position) {
                this.position = position;
            }
        }
    }

    private final SparkMax elevatorMotor = new SparkMax(CAN.CANID, MotorType.kBrushless);
    private final SparkMaxConfig elevatorConfig = new SparkMaxConfig();
    private final RelativeEncoder elevatorEncoder;
    private final TrapezoidProfile.Constraints elevatorProfile = Constants.ELEVATOR_PROFILE;
    private final ProfiledPIDController elevatorPidController = new ProfiledPIDController(
        Feedback.kP, 
        Feedback.kI, 
        Feedback.kD, 
        elevatorProfile
    );

    private final ElevatorFeedforward elevatorFeedforward = new ElevatorFeedforward(
        Feedforward.kS,
        Feedforward.kG,
        Feedforward.kV,
        Feedforward.kA
    );

    private double elevatorVoltage;

    private final MutVoltage sysIdVoltage = Volts.mutable(0);
    private final MutDistance sysIdPosition = Meters.mutable(0);
    private final MutLinearVelocity sysIdVelocity = MetersPerSecond.mutable(0);

    private final SysIdRoutine sysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(),
        new SysIdRoutine.Mechanism(
            (voltage)->{
                setVoltage(voltage.magnitude());
            }, 
            (log)->{
                log.motor("elevator")
                    .voltage(sysIdVoltage.mut_replace(getVoltage(), Volts))
                    .linearPosition(sysIdPosition.mut_replace(getPosition(), Meters))
                    .linearVelocity(sysIdVelocity.mut_replace(getVelocity(), MetersPerSecond));
            }, 
            this
        )
    );
    
    public Elevator(){
        elevatorConfig.smartCurrentLimit(MotorConfigs.CURRENT_LIMIT);
        elevatorConfig.encoder.positionConversionFactor(MotorConfigs.ENCODER_CONVERSION_FACTOR);
        elevatorConfig.inverted(MotorConfigs.INVERTED);

        elevatorMotor.configure(elevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        elevatorEncoder = elevatorMotor.getEncoder();

        elevatorPidController.setTolerance(Feedback.TOLERANCE);

        setDefaultCommand(moveToCurrentGoalCommand());
    }

    @Override
    public double getPosition() {
        return elevatorEncoder.getPosition();
    }

    @Override
    public void resetPosition() {
        elevatorEncoder.setPosition(Position.RESET.position);
    }

    public double getVoltage(){
        return elevatorMotor.getAppliedOutput();
    }

    public double getVelocity(){
        return elevatorEncoder.getVelocity();
    }

    public boolean atGoal(){
        return elevatorPidController.atGoal();
    }

    @Override
    public void setVoltage(double voltage) {
        elevatorMotor.setVoltage(MathUtil.clamp(voltage,-12,12));
    }

    @Override
    public Command moveToCurrentGoalCommand() {
        return run(()->{
            double feedforward = elevatorFeedforward.calculate(elevatorPidController.getSetpoint().velocity);
            double pid = elevatorPidController.calculate(getPosition());
            elevatorVoltage = MathUtil.clamp(feedforward+pid, -12, 12);
            setVoltage(elevatorVoltage);
        }).withName("elevator.moveToCurrentGoal");
        
    }

    @Override
    public Command moveToPositionCommand(Supplier<Position> goalPositionSupplier) {
        return Commands.sequence(
            runOnce(()-> elevatorPidController.reset(getPosition())),
            runOnce(()-> elevatorPidController.setGoal(goalPositionSupplier.get().position)),
            moveToCurrentGoalCommand().until(this::atGoal).withName("elevator.moveToPosition")
        );
    }

    @Override
    public Command moveToArbitraryPositionCommand(Supplier<Double> goalPositionSupplier) {
        return Commands.sequence(
            runOnce(()-> elevatorPidController.reset(getPosition())),
            runOnce(()-> elevatorPidController.setGoal(goalPositionSupplier.get())),
            moveToCurrentGoalCommand().until(this::atGoal).withName("elevator.moveToArbitraryPosition")
        );
    }

    @Override
    public Command movePositionDeltaCommand(Supplier<Double> delta) {
        return moveToArbitraryPositionCommand(()->getPosition()+delta.get())
            .withName("elevator.movePositionDelta");
    }

    @Override
    public Command holdCurrentPositionCommand() {
        return runOnce(()->elevatorPidController.setGoal(getPosition())).andThen(moveToCurrentGoalCommand())
            .withName("elevator.holdCurrentPosition");
    }

    @Override
    public Command resetPositionCommand() {
        return runOnce(this::resetPosition).withName("elevator.resetPosition");
    }

    @Override
    public Command setOverridenSpeedCommand(Supplier<Double> speed) {
        return runEnd(()->setVoltage(speed.get()*12.0),()->setVoltage(0))
            .withName("elevator.setOverridenSpeed");
    }

    @Override
    public Command coastMotorsCommand() {
        return runOnce(elevatorMotor::stopMotor)
            .andThen(()->{
                elevatorConfig.idleMode(IdleMode.kCoast);
                elevatorMotor.configure(elevatorConfig, ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
            }).finallyDo((d)->{
                elevatorConfig.idleMode(IdleMode.kBrake);
                elevatorMotor.configure(elevatorConfig, ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
            }).withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
            .withName("elevator.coastMotors");
    }
}
