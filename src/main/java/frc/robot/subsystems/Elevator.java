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

    public Elevator(){
        elevatorConfig.smartCurrentLimit(MotorConfigs.CURRENT_LIMIT);
        elevatorConfig.encoder.positionConversionFactor(MotorConfigs.ENCODER_CONVERSION_FACTOR);
        elevatorConfig.inverted(MotorConfigs.INVERTED);

        elevatorMotor.configure(elevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        elevatorEncoder = elevatorMotor.getEncoder();
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
