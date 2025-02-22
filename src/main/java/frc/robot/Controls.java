package frc.robot;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.Climber;

import java.util.concurrent.locks.Condition;

import com.techhounds.houndutil.houndlib.oi.CommandVirpilJoystick;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

public class Controls {
    public static final class Constants {
        public static String CURRENT_USE = "SysId"; //"Driving" if we want to drive
    }
    private static boolean inCoralMode = true; //set to coral mode as default
    private static boolean alignLeft = false; //align first to the right
    private static int level = 1;
    private static boolean isClimbing = false;


    private static void configureOperatorControls(int port, Drivetrain drivetrain, CoralIntake coralintake, Elevator elevator, AlgaeIntake algaeintake, Climber climber){
        CommandXboxController controller = new CommandXboxController(port);
        ConditionalCommand runIntakeCommand = new ConditionalCommand(algaeintake.runRollersCommand(), coralintake.runRollersCommand(), () -> controller.getHID().getRightBumperButton());
        ConditionalCommand runOuttake = new ConditionalCommand(algaeintake.reverseRollersCommand(), coralintake.reverseRollersCommand(), () -> controller.getHID().getRightBumperButton());
        
        switch (Constants.CURRENT_USE) {
            case "Driving":
                controller.b().whileTrue(runIntakeCommand);
                controller.a().whileTrue(runOuttake);
                
                controller.povUp().whileTrue(elevator.movePositionDeltaCommand(() -> 0.5));
                controller.povDown().whileTrue(elevator.movePositionDeltaCommand(() -> 0.5));

                controller.x().onTrue(climber.movePositionDeltaCommand(() -> 0.5));

                drivetrain.setDefaultCommand(drivetrain.teleopDriveCommand(controller::getLeftX, controller::getRightY, () -> 0));
                //arbitrary values 0.5 need refining
            case "SysId":
                controller.x().and(controller.povUp()).whileTrue(drivetrain.sysIdDriveQuasistatic(Direction.kForward));
                controller.y().and(controller.povUp()).whileTrue(drivetrain.sysIdDriveQuasistatic(Direction.kReverse));
                controller.a().and(controller.povUp()).whileTrue(drivetrain.sysIdDriveDynamic(Direction.kForward));
                controller.b().and(controller.povUp()).whileTrue(drivetrain.sysIdDriveDynamic(Direction.kReverse));
        }  
    }




}
