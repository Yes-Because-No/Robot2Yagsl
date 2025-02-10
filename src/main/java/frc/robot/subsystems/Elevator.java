package frc.robot.subsystems;

import java.util.function.Supplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.techhounds.houndutil.houndlib.subsystems.BaseLinearMechanism;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Elevator.Constants.*;

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

    public Elevator(){
        elevatorConfig.smartCurrentLimit(MotorConfigs.CURRENT_LIMIT);
        elevatorConfig.encoder.positionConversionFactor(MotorConfigs.ENCODER_CONVERSION_FACTOR);
        elevatorConfig.inverted(MotorConfigs.INVERTED);

        elevatorMotor.configure(elevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        elevatorEncoder = elevatorMotor.getEncoder();

        elevatorPidController.setTolerance(Feedback.TOLERANCE);

    }

    @Override
    public double getPosition() {
        return elevatorEncoder.getPosition();
    }

    @Override
    public void resetPosition() {
        elevatorEncoder.setPosition(Position.RESET.position);
    }

    @Override
    public void setVoltage(double voltage) {
        elevatorMotor.setVoltage(MathUtil.clamp(voltage,-12,12));
    }

    @Override
    public Command moveToCurrentGoalCommand() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'moveToCurrentGoalCommand'");
    }

    @Override
    public Command moveToPositionCommand(Supplier<Position> goalPositionSupplier) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'moveToPositionCommand'");
    }

    @Override
    public Command moveToArbitraryPositionCommand(Supplier<Double> goalPositionSupplier) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'moveToArbitraryPositionCommand'");
    }

    @Override
    public Command movePositionDeltaCommand(Supplier<Double> delta) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'movePositionDeltaCommand'");
    }

    @Override
    public Command holdCurrentPositionCommand() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'holdCurrentPositionCommand'");
    }

    @Override
    public Command resetPositionCommand() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'resetPositionCommand'");
    }

    @Override
    public Command setOverridenSpeedCommand(Supplier<Double> speed) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setOverridenSpeedCommand'");
    }

    @Override
    public Command coastMotorsCommand() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'coastMotorsCommand'");
    }
}
