package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Volts;

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
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
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
            public static final int ARM_L = 0; // TODO
            public static final int ARM_R = 0; // TODO
            public static final int BAR = 0; // TODO
        }

        public static final class INVERSION {
            public static final boolean ARM_L = true; // TODO after testing
            public static final boolean ARM_R = false; // TODO after testing
            public static final boolean BAR = false; // TODO after testing
        }

        public static final class GEAR_RATIOS {
            public static final double ARM = (4.0 / 1.0);
            public static final double BAR = (3.0 / 1.0);
        }

        public static final class Feedback {
            public static final double kP = 0; // TODO SysId
            public static final double kI = 0; // TODO SysId
            public static final double kD = 0; // TODO SysId
            public static final double MAX_VELOCITY = 0; // TODO SysId
            public static final double MAX_ACCEL = 0; // TODO SysId
        }

        public static final class Feedforward {
            public static final double kS = 0; // TODO SysId
            public static final double kG = 0; // TODO SysId
            public static final double kV = 0; // TODO SysId
            public static final double kA = 0; // TODO SysId
        }

        public static final double VOLTAGE = 0; // TODO test for desired speed of bar

        public static enum Position {
            ARM_RESET(0.0); // TODO test for actual position

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
    private final ProfiledPIDController PID = new ProfiledPIDController(Constants.Feedback.kP, Constants.Feedback.kI,
        Constants.Feedback.kD,
        new TrapezoidProfile.Constraints(Constants.Feedback.MAX_VELOCITY, Constants.Feedback.MAX_ACCEL));
    private final ArmFeedforward Feedforward = new ArmFeedforward(Constants.Feedforward.kS, Constants.Feedforward.kG,
        Constants.Feedforward.kV, Constants.Feedforward.kA);

    // SysId stuffs
    private final MutVoltage sysIdVoltage = Volts.mutable(0);
    private final MutAngle sysIdAngle = Degrees.mutable(0);
    private final MutAngularVelocity sysIdAngularVelocity = DegreesPerSecond.mutable(0);

    private final SysIdRoutine sysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(),
        new SysIdRoutine.Mechanism(
            (voltage) -> {
                setVoltage(voltage.magnitude());
            },
            (log) -> {
                log.motor("AlgaeIntakeArm").voltage(sysIdVoltage.mut_replace(getVoltage(), Volts))
                    .angularPosition(sysIdAngle.mut_replace(getPosition(), Degrees))
                    .angularVelocity(sysIdAngularVelocity.mut_replace(getVelocity(), DegreesPerSecond));
            },
            this
        )
    );

    /**
     * The constructor for the coral intake subsystem
     * 
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
            .idleMode(IdleMode.kCoast)
            .inverted(Constants.INVERSION.BAR);

        armL.configure(armLConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        armR.configure(armRConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        bar.configure(barConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        PID.reset(getPosition());
    }

    /**
     * Gets the position of the algae intake arm mechanism
     * 
     * @return the position in radians
     */
    @Override
    public double getPosition() {
        return armLEncoder.getPosition();
    }

    /**
     * Gets the velocity of the {@link #armLEncoder encoder}
     * 
     * @return The velocity, in degrees per second, as a double
     */
    public double getVelocity() {
        return armLEncoder.getVelocity();
    }

    /**
     * Gets the voltage applied to the {@link #armL motor}
     * 
     * @return voltage applied to the motor, as a double
     */
    public double getVoltage() {
        return armL.getAppliedOutput();
    }

    /** Resets the encoder positions of the algae intake arm mechanism */
    @Override
    public void resetPosition() {
        armLEncoder.setPosition(Constants.Position.ARM_RESET.position);
        armREncoder.setPosition(Constants.Position.ARM_RESET.position);
    }

    /**
     * Explicitly set the voltage of the algae intake arm mechanism
     * 
     * @param voltage to set [-12, 12]
     */
    @Override
    public void setVoltage(double voltage) {
        armL.setVoltage(MathUtil.clamp(voltage, -12, 12));
        armR.setVoltage(MathUtil.clamp(voltage, -12, 12));
    }

    /**
     * Move the algae intake arm mechanism to it's current goal position
     * 
     * @return the command
     */
    @Override
    public Command moveToCurrentGoalCommand() {
        return run(() -> {
            setVoltage(PID.calculate(getPosition())
                + Feedforward.calculate(PID.getGoal().position, PID.getGoal().velocity));
        }).withName("AlgaeIntake.moveToCurrentGoalCommand");
    }

    /**
     * Move the algae intake arm mechanism to a defined position by
     * setting the goal and moving to the goal until reached
     * 
     * @param goalPositionSupplier a supplier of an instance of the position enum
     * @return the command
     */
    @Override
    public Command moveToPositionCommand(Supplier<Position> goalPositionSupplier) {
        return Commands.sequence(
            runOnce(() -> {
                PID.setGoal(goalPositionSupplier.get().position);
            }),
            moveToCurrentGoalCommand().until(() -> PID.atGoal())
        ).withName("AlgaeIntake.moveToPositionCommand");
    }

    /**
     * Move the algae intake arm mechanism to any position by
     * setting the goal and moving to the goal until reached
     * 
     * @param goalPositionSupplier a supplier of the position in radians
     * @return the command
     */
    @Override
    public Command moveToArbitraryPositionCommand(Supplier<Double> goalPositionSupplier) {
        return Commands.sequence(
            runOnce(() -> {
                PID.setGoal(goalPositionSupplier.get());
            }),
            moveToCurrentGoalCommand().until(() -> PID.atGoal())
        ).withName("AlgaeIntake.moveToArbitraryPositionCommand");
    }

    /**
     * Move the algae intake arm mechanism to the current position plus a delta by
     * setting the goal and moving to the goal until reached
     * 
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
        ).withName("AlgaeIntake.movePositionDeltaCommand");
    }

    /**
     * Hold the algae intake arm mechanism at it's current position by
     * setting the goal to the current goal and moving toward it until cancelled
     * 
     * @return the command
     */
    @Override
    public Command holdCurrentPositionCommand() {
        return Commands.sequence(
            runOnce(() -> {
                PID.setGoal(PID.getGoal().position);
            }),
            moveToCurrentGoalCommand()
        ).withName("AlgaeIntake.holdCurrentPositionCommand");
    }

    /**
     * Instantly reset the encoder positions of the algae intake arm mechanism
     * 
     * @return the command
     */
    @Override
    public Command resetPositionCommand() {
        return runOnce(() -> {
            resetPosition();
        }).withName("resetPositionCommand");
    }

    /**
     * Explicitly set the speed of the algae intake arm mechanism,
     * overridding PID & feedforward control
     * 
     * @param speed the speed [-1, 1]
     * @return the command
     */
    @Override
    public Command setOverridenSpeedCommand(Supplier<Double> speed) {
        return runEnd(() -> {
            setVoltage(speed.get() * 12);
        }, () -> {
            setVoltage(0);
        }).withName("AlgaeIntake.setOverridenSpeedCommand");
    }

    /**
     * Coast the motors of the robot for manual movement by
     * stopping them, setting them to coast, and then setting them back to break on
     * command end
     * 
     * @return the command
     */
    @Override
    public Command coastMotorsCommand() {
        return runOnce(() -> {
            armL.stopMotor();
            armR.stopMotor();
        }).andThen(() -> {
            armLConfig.idleMode(IdleMode.kCoast);
            armRConfig.idleMode(IdleMode.kCoast);
            armL.configure(armLConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
            armR.configure(armRConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        }).finallyDo(() -> {
            armLConfig.idleMode(IdleMode.kBrake);
            armRConfig.idleMode(IdleMode.kBrake);
            armL.configure(armLConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
            armR.configure(armRConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        });
    }

    /**
     * Run the bar of the algae intake at a predetermined voltage in the direction
     * that will intake an algae
     * 
     * @return the command
     */
    @Override
    public Command runRollersCommand() {
        return startEnd(() -> {
            bar.setVoltage(Constants.VOLTAGE);
        }, () -> {
            bar.stopMotor();
        }).withName("AlgaeIntake.runRollersCommand");
    }

    /**
     * Run the bar of the algae intake at a predetermined voltage in the direction
     * that will eject an algae
     * 
     * @return the command
     */
    @Override
    public Command reverseRollersCommand() {
        return startEnd(() -> {
            bar.setVoltage(-Constants.VOLTAGE);
        }, () -> {
            bar.stopMotor();
        }).withName("AlgaeIntake.reverseRollersCommand");
    }

    /**
     * Creates a command for the sysId quasistatic test, which gradually speeds up
     * the mechanism to eliminate variation from acceleration
     * 
     * @see https://docs.wpilib.org/en/stable/docs/software/advanced-controls/system-identification/creating-routine.html
     * @param direction Direction to run the motors in
     * @return Command that runs the quasistatic test
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }

    /**
     * Creates a command for the sysId dynamic test, which will step up the speed to
     * see how the mechanism behaves during acceleration
     * 
     * @see https://docs.wpilib.org/en/stable/docs/software/advanced-controls/system-identification/creating-routine.html
     * @param direction Direction to run the motors in
     * @return Command that runs the dynamic test
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }
}