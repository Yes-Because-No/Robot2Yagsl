package frc.robot;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.Climber;

import com.techhonds.houndutil.houndlib.oi.CommandVirpilJoystick;


public class Controls {
    
    private static boolean inCoralMode = true; //set to coral mode as default
    private static boolean alignLeft = false; //align first to the right
    private static int level = 1;
    private static boolean isClimbing = false;

}
