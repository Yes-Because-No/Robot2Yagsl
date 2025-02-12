package frc.robot.subsystems;

import java.util.function.Supplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.techhounds.houndutil.houndlib.subsystems.BaseIntake;
import com.techhounds.houndutil.houndlib.subsystems.BaseSingleJointedArm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.AlgaeIntake.Constants.Position;

/** The subsystem for the robot's algae intake and scoring mechanism */
public class AlgaeIntake extends SubsystemBase implements BaseIntake, BaseSingleJointedArm<Position> {
    public static final class Constants {
        public static final class CAN_IDS {
            public static final int ARM_L = 12;
            public static final int ARM_R = 17;
            public static final int BAR = 18;
        }

        public static final class CURRENT_LIMITS {
            public static final int ARM_L = 0; //TODO
            public static final int ARM_R = 0; //TODO
            public static final int BAR = 0; //TODO
        }

        public static final class INVERSION {
            public static final boolean ARM_L = true; //TODO after testing
            public static final boolean ARM_R = false; //TODO after testing
            public static final boolean BAR = false; //TODO after testing
        }

        public static final class GEAR_RATIOS {
            public static final double ARM = 0; //TODO get actual gear ratio
            public static final double BAR = 0; //TODO get actual gear ratio
        }

        public static final class Feedback {
            public static final double kP = 0; //TODO SysId
            public static final double kI = 0; //TODO SysId
            public static final double kD = 0; //TODO SysId
            public static final double MAX_VELOCITY = 0; //TODO SysId
            public static final double MAX_ACCEL = 0; //TODO SysId
        }

        public static final class Feedforward {
            public static final double kS = 0; //TODO SysId
            public static final double kG = 0; //TODO SysId
            public static final double kV = 0; //TODO SysId
        }

        public static enum Position {
            ARM_RESET(0.0); //TODO test for actual position

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

    // Create encoder objects
    private final RelativeEncoder armLEncoder = armL.getEncoder();
    private final RelativeEncoder armREncoder = armR.getEncoder();

    // Create config objects
    private final SparkMaxConfig armLConfig = new SparkMaxConfig();
    private final SparkMaxConfig armRConfig = new SparkMaxConfig();
    private final SparkMaxConfig barConfig = new SparkMaxConfig();

    // Create controller objects
    private final ProfiledPIDController PID = new ProfiledPIDController(Constants.Feedback.kP, Constants.Feedback.kI, Constants.Feedback.kD, 
        new TrapezoidProfile.Constraints(Constants.Feedback.MAX_VELOCITY, Constants.Feedback.MAX_ACCEL));
    private final ArmFeedforward Feedforward = new ArmFeedforward(Constants.Feedforward.kS, Constants.Feedforward.kG, Constants.Feedforward.kV);

    /** The constructor for the coral intake subsystem
     * @return the object
     */
    public AlgaeIntake() {
        // Configure each motor & encoder
        armLConfig
            .smartCurrentLimit(Constants.CURRENT_LIMITS.ARM_L)
            .idleMode(IdleMode.kBrake)
            .inverted(Constants.INVERSION.ARM_L)
            .encoder
                .positionConversionFactor(Constants.GEAR_RATIOS.ARM * (2 * Math.PI))
                .velocityConversionFactor((Constants.GEAR_RATIOS.ARM * (2 * Math.PI)) / 60);
        
        armRConfig
            .smartCurrentLimit(Constants.CURRENT_LIMITS.ARM_R)
            .idleMode(IdleMode.kBrake)
            .inverted(Constants.INVERSION.ARM_R)
            .encoder
                .positionConversionFactor(Constants.GEAR_RATIOS.ARM * (2 * Math.PI))
                .velocityConversionFactor((Constants.GEAR_RATIOS.ARM * (2 * Math.PI)) / 60);

        barConfig
            .smartCurrentLimit(Constants.CURRENT_LIMITS.BAR)
            .idleMode(IdleMode.kBrake)
            .inverted(Constants.INVERSION.BAR)
            .encoder
                .positionConversionFactor(Constants.GEAR_RATIOS.BAR * (2 * Math.PI))
                .velocityConversionFactor((Constants.GEAR_RATIOS.BAR * (2 * Math.PI)) / 60);

        armL.configure(armLConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        armR.configure(armRConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        bar.configure(barConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /** Gets the position of the algae intake arm mechanism
     * @return the position in radians
     */
    @Override
    public double getPosition() {
        return armLEncoder.getPosition();
    }

    /** Resets the encoder positions of the algae intake arm mechanism */
    @Override
    public void resetPosition() {
        armLEncoder.setPosition(Constants.Position.ARM_RESET.position);
        armREncoder.setPosition(Constants.Position.ARM_RESET.position);
    }

    /** Explicitly set the voltage of the algae intake arm mechanism 
     * @param voltage to set [-12, 12]
    */
    @Override
    public void setVoltage(double voltage) {
        armL.setVoltage(MathUtil.clamp(voltage, -12, 12));
        armR.setVoltage(MathUtil.clamp(voltage, -12, 12));
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
