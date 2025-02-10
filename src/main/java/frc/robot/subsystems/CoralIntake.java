package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.techhounds.houndutil.houndlib.subsystems.BaseIntake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** The subsystem for the robot's coral intake and scoring mechanism */
public class CoralIntake extends SubsystemBase implements BaseIntake {
    public static final class Constants {
        //CAN IDS for Spark Maxes
        public static final class CAN_IDS{
            public static int MOTOR_L = 20;
            public static int MOTOR_R = 21;

        }
        public static final double VOLTAGE = 0; //ToDo: Find the correct voltage
        public static final class INVERSIONS {
            public static final boolean MOTOR_L = true;//ToDo: Find the correct Inversions
            public static final boolean MOTOR_R = false;
        }
        public static final class CURRENT_LIMIT{
            public static final int MOTOR_L = 0;//ToDo: Find the correct current limit
            public static final int MOTOR_R = 0;

        }
    }
    
    private static SparkMax motorL = new SparkMax(Constants.CAN_IDS.MOTOR_L, MotorType.kBrushless);
    private static SparkMaxConfig motorLConfig = new SparkMaxConfig();

    /** The constructor for the coralIntake subsystem
     * @return the object
    */
    public CoralIntake(){
        motorLConfig
        .smartCurrentLimit(Constants.CURRENT_LIMIT.MOTOR_L)
        .inverted(Constants.INVERSIONS.MOTOR_L);

        motorRConfig
        .smartCurrentLimit(Constants.CURRENT_LIMIT.MOTOR_R)
        .inverted(Constants.INVERSIONS.MOTOR_R);
    }
    private static SparkMax motorR = new SparkMax(Constants.CAN_IDS.MOTOR_R, MotorType.kBrushless);
    private static SparkMaxConfig motorRConfig = new SparkMaxConfig();

    /** Runs the rollers of the coral intake mechanism forward 
     * @return the command
    */
    @Override
    public Command runRollersCommand() {
        return startEnd (() ->{
            motorL.setVoltage(Constants.VOLTAGE);
            motorR.setVoltage(Constants.VOLTAGE);
        }, () -> {
            motorL.stopMotor();
            motorR.stopMotor();
        }).withName("CoralIntake.runRollersCommand");

    }
    /** Runs the rollers of the coral intake mechanism in reverse
    * @return the command
    */
    @Override
    public Command reverseRollersCommand() {
        return startEnd (() ->{
            motorL.setVoltage(-Constants.VOLTAGE);
            motorR.setVoltage(-Constants.VOLTAGE);
        }, () -> {
            motorL.stopMotor();
            motorR.stopMotor();
        }).withName("CoralIntake.reverseRollersCommand");
    }
}
