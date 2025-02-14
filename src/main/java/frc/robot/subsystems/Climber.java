package frc.robot.subsystems;

import java.util.function.Supplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.techhounds.houndutil.houndlib.subsystems.BaseIntake;
import com.techhounds.houndutil.houndlib.subsystems.BaseSingleJointedArm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Climber.Constants.Position;

/** The subsystem for the robot's climber mechanism */
public class Climber extends SubsystemBase implements BaseIntake, BaseSingleJointedArm<Position> {

    public static final class Constants {
        public static final class CAN_IDS {
            public static final int ARM = 7;
            public static final int JAW = 8; 
        }
        public static final double VOLTAGE = 0; //TODO: Find the correct voltage

        public static final class CURRENT_LIMITS {
            public static final int ARM = 0;//TODO
            public static final int JAW = 0;
        }

        public static final class INVERSION {
            public static final boolean ARM = false;//TODO
            public static final boolean JAW = false;
        }

        public static final class GEAR_RATIO {
            public static final double ARM = 12/1;
            public static final double JAW = 4/1;
        }

        public static final class feedBackward {
            public static final double kMaxVelocity = 0;//TODO
            public static final double kMaxAcceleration = 0;
            public static final double kP = 0;
            public static final double kI = 0;
            public static final double kD = 0;
        }

        public static final class feedForward {
            public static final double kS = 0;//TODO
            public static final double kG = 0;
            public static final double kV = 0;
            public static final double kA = 0;
        }
        //I moved position into constants and it fixed stuff but i dont know if it fixed right
        public static enum Position {
            ARM_RESET(0.0);//TODO

            public final double position;

            private Position(final double position) {
                this.position = position;
            }

        }

    }
    //Creates motor objects
    private final SparkMax arm = new SparkMax(Constants.CAN_IDS.ARM, MotorType.kBrushless);
    private final SparkMax jaw = new SparkMax(Constants.CAN_IDS.JAW, MotorType.kBrushless);

    //Creates encoder objects
    private final RelativeEncoder armEncoder = arm.getEncoder();
    
    //Create config objects
    private final SparkMaxConfig armConfig = new SparkMaxConfig();
    private final SparkMaxConfig jawConfig = new SparkMaxConfig();

    //Create controller objects
    private final TrapezoidProfile.Constraints Constraints =
      new TrapezoidProfile.Constraints(Constants.feedBackward.kMaxVelocity, Constants.feedBackward.kMaxAcceleration);
  private final ProfiledPIDController PID =
      new ProfiledPIDController(Constants.feedBackward.kP, Constants.feedBackward.kI, Constants.feedBackward.kD, Constraints);
  private final ArmFeedforward feedforward = 
      new ArmFeedforward(Constants.feedForward.kS, Constants.feedForward.kG, Constants.feedForward.kV, Constants.feedForward.kA);

    //Constructor
    public Climber(){
        armConfig
        .smartCurrentLimit(Constants.CURRENT_LIMITS.ARM)
        .inverted(Constants.INVERSION.ARM)
        .idleMode(IdleMode.kBrake)
        .encoder
            .positionConversionFactor(Constants.GEAR_RATIO.ARM * (2 * Math.PI))
            .velocityConversionFactor(Constants.GEAR_RATIO.ARM * (2 * Math.PI));

        jawConfig
        .smartCurrentLimit(Constants.CURRENT_LIMITS.JAW)
        .inverted(Constants.INVERSION.JAW);

        //Apply configurations
        arm.configure(armConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        jaw.configure(jawConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    
    /** Get position of arm
     * @return arm position in radians.
     */
    @Override
    public double getPosition() {
        return armEncoder.getPosition();
    }
    /**
     * Reset position of arm to reset position
     */
    @Override
    public void resetPosition() {
        armEncoder.setPosition(Constants.Position.ARM_RESET.position);
    }
    /**Sets voltage within minimum and maximum
     * @param voltage [-12.12]
     */
    @Override
    public void setVoltage(double voltage) {
        arm.setVoltage(MathUtil.clamp(voltage, -12, 12));

    }
    /**Move climber arm toward current goal 
     * @return the command
    */
    @Override
    public Command moveToCurrentGoalCommand() {
        return run(() -> {
            setVoltage(PID.calculate(getPosition()) + feedforward.calculate(PID.getGoal().position, PID.getGoal().velocity));
        }).withName("Climber.moveToCurrentGoalCommand");
    }
    /**Moves arm to the preset position and sets the goal until it is reached
     * @param goalPositionSupplier supplies position of enum 
     * @return the command
     */
    @Override
    public Command moveToPositionCommand(Supplier<Position> goalPositionSupplier) {
        return Commands.sequence(
            runOnce(() -> {
                PID.setGoal(goalPositionSupplier.get().position);
            }),
            moveToCurrentGoalCommand().until(()-> PID.atGoal())
        ).withName("Climber.moveToPositionCommand");
    }
    /**Moves arm to position that is set, setting the goal until it's reached
     * @param goalPositionSupplier supplies posiiton in radians
     * @return the command
     */
    @Override
    public Command moveToArbitraryPositionCommand(Supplier<Double> goalPositionSupplier) {
        return Commands.sequence(
            runOnce(() -> {
                PID.setGoal(goalPositionSupplier.get());
            }),
            moveToCurrentGoalCommand().until(() -> PID.atGoal())
            ).withName("Climber.moveToArbitraryPositionCommand");
    }
    /**Moves arm to current position plus a delta
     * sets a goal and moves it to the goal until it is reached
     * @param delta a supplier of the delta in radians
     * @return the command
    */
    @Override
    public Command movePositionDeltaCommand(Supplier<Double> delta) {
        return Commands.sequence(
            runOnce(() -> {
                PID.setGoal(PID.getGoal().position + delta.get());
            }),
            moveToCurrentGoalCommand().until(() -> PID.atGoal())
        ).withName("Climber.movePositionDeltaCommand");
    }
    /**Keeps arm at its position by setting the goal and moving to the goal
     * @return the command
     */
    @Override
    public Command holdCurrentPositionCommand() {
        return Commands.sequence(
            runOnce(() -> {
                PID.setGoal(PID.getGoal().position);
            }),
            moveToCurrentGoalCommand()
            ).withName("Climber.holdCurrentPositionCommand");
        }
    
    /**Resets encoder positions to reset state.
     * @return the command
     */
    @Override
    public Command resetPositionCommand() {
        return runOnce(()->{
            resetPosition();
        }).withName("Climber.resetPositionCommand");
    }
    /**Explicity sets speed of climber arm 
     * overrides pid and feedforward
     * @param speed the speed [-1, 1]
     * @return the command
     */
    @Override
    public Command setOverridenSpeedCommand(Supplier<Double> speed) {
        return runEnd(() -> {
            setVoltage(speed.get() * 12);
        },() -> {
            setVoltage(0);
        }).withName("Climber.setOverridenSpeedCommand");
    }
    /** Coast the motors of the robot for manual movement by
     * stopping them, setting them to coast, and then setting them back to break on command end
     * @return the command
     */
    @Override
    public Command coastMotorsCommand() {
        return runOnce(()->{
            arm.stopMotor();
        }).andThen(()->{
            armConfig.idleMode(IdleMode.kCoast);
            arm.configure(armConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        }).finallyDo(()->{
            armConfig.idleMode(IdleMode.kBrake);
            arm.configure(armConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        }).withName("Climber.coastMo    torsCommand");
    }

    @Override
    /** Runs the rollers of the coral intake mechanism forwards
    * @return the command
    */
    public Command runRollersCommand() {
        return startEnd (() ->{
            jaw.setVoltage(Constants.VOLTAGE);
        }, () -> {
            jaw.stopMotor();
        }).withName("Climber.runRollersCommand");
    }

    @Override
    /** Runs the rollers of the coral intake mechanism in reverse
    * @return the command
    */
    public Command reverseRollersCommand() {
        return startEnd (() ->{
            jaw.setVoltage(-Constants.VOLTAGE);
        }, () -> {
            jaw.stopMotor();
        }).withName("Climber.reverseRollersCommand");
    }
}
