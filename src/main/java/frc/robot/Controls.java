package frc.robot;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.Climber;

import com.techhounds.houndutil.houndlib.oi.CommandVirpilJoystick;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


public class Controls {
    
    private static boolean inCoralMode = true; //set to coral mode as default
    private static boolean alignLeft = false; //align first to the right
    private static int level = 1;
    private static boolean isClimbing = false;

    private static void configureOperatorControls(int port, Drivetrain drivetrain, CoralIntake coralintake, Elevator elevator, AlgaeIntake algaeintake, Climber climber){
        
        CommandXboxController controller = new CommandXboxController(port);
        
        
        
    }




}
