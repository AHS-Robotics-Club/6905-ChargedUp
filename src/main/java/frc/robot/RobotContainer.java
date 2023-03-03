// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.IOConstants;
import frc.robot.subsystems.DriveSubsystem;

public class RobotContainer {

  private final DriveSubsystem driveSubsystem = new DriveSubsystem();

  CommandXboxController controller = new CommandXboxController(IOConstants.DRIVER_CONTROLLER_PORT_1);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    // I <3 my Lean-ona :)

    driveSubsystem.setDefaultCommand(Commands.run(
      () -> driveSubsystem.arcadeDrive(-controller.getLeftY() * 0.5, controller.getRightX() * 0.5),
      driveSubsystem
    ));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    // controller.leftBumper()
    //   .onTrue(Commands.run(() -> driveSubsystem.setToSlowOutput()))
    //   .onFalse(Commands.run(() -> driveSubsystem.setToMaxOutput()));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return Commands.print("No autonomous command");
  }
}