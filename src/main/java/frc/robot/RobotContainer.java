// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.IOConstants;
import frc.robot.autons.CollectCubeTMBlue;
import frc.robot.subsystems.DriveSubsystem;

public class RobotContainer {

  // Robot subsystems
  private final DriveSubsystem driveSubsystem = new DriveSubsystem();

  // Controller
  private final CommandXboxController controller = 
    new CommandXboxController(IOConstants.DRIVER_CONTROLLER_PORT_1);

  public RobotContainer() {
    configureBindings();

    // Sets default command for drive subsystem
    driveSubsystem.setDefaultCommand(new RunCommand(
      () -> driveSubsystem.arcadeDrive(-controller.getLeftY(), controller.getRightX()),
      driveSubsystem
    ));
  }

  // Sets all controller keybindings
  private void configureBindings() {

    // Slow mode on left bumper button 
    // controller.leftBumper()
    //   .onTrue(Commands.runOnce(() -> driveSubsystem.setToSlowOutput()))
    //   .onFalse(Commands.runOnce(() -> driveSubsystem.setToMaxOutput()));

  }

  // Calls our autonomous command
  public Command getAutonomousCommand() {

    // Test Auton with custom classes
    // return new PrintCommand("Auton not being made.");

    return new CollectCubeTMBlue(driveSubsystem);

  }
}
