package frc.robot.subsystems;

import java.util.function.Supplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.techhounds.houndutil.houndlib.subsystems.BaseIntake;
import com.techhounds.houndutil.houndlib.subsystems.BaseSingleJointedArm;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.AlgaeIntake.Constants.Position;

/** The subsystem for the robot's algae intake and scoring mechanism */
public class AlgaeIntake extends SubsystemBase implements BaseIntake, BaseSingleJointedArm<Position> {
    public static final class Constants {
        public static final class CAN_IDS {
            public static final int ARM_L = 0; //TODO when CAN finalized
            public static final int ARM_R = 0; //TODO when CAN finalized
            public static final int BAR = 0; //TODO when CAN finalized
        }
        public static enum Position {
            ZERO(0.0),
            INTAKE(0.0);

            public final double position;

            private Position(final double position) {
                this.position = position;
            }
        }
    }

    // Create motor objects
    private final SparkMax armL = new SparkMax(Constants.CAN_IDS.ARM_L, MotorType.kBrushless);
    private final SparkMax armR = new SparkMax(Constants.CAN_IDS.ARM_R, MotorType.kBrushless);
    private final SparkMax bar = new SparkMax(Constants.CAN_IDS.BAR, MotorType.kBrushless);

    //Create encoder objects
    private final RelativeEncoder armLEncoder = armL.getEncoder();
    private final RelativeEncoder armREncoder = armR.getEncoder();
    private final RelativeEncoder barEncoder = bar.getEncoder();

    //Create config objects
    private final SparkMaxConfig armLConfig = new SparkMaxConfig();
    private final SparkMaxConfig armRConfig = new SparkMaxConfig();
    private final SparkMaxConfig barConfig = new SparkMaxConfig();

    @Override
    public double getPosition() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getPosition'");
    }

    @Override
    public void resetPosition() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'resetPosition'");
    }

    @Override
    public void setVoltage(double voltage) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setVoltage'");
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

    @Override
    public Command runRollersCommand() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'runRollersCommand'");
    }

    @Override
    public Command reverseRollersCommand() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'reverseRollersCommand'");
    }
}
