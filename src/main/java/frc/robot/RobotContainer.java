// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;

public class RobotContainer {
  private final SwerveSubsystem drivetrain = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
  private final CommandXboxController controller = new CommandXboxController(0);

  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(
      drivetrain.getSwerveDrive(),
      () -> controller.getLeftY() * -1,
      () -> controller.getLeftX() * -1)
      .withControllerRotationAxis(controller::getRightX)
      .deadband(0.2)
      .scaleTranslation(1)
      .scaleRotation(1)
      .allianceRelativeControl(true);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    drivetrain.setDefaultCommand(drivetrain.driveFieldOriented(driveAngularVelocity));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
