// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.IOConstants;
import frc.robot.commands.IntakeCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LiftSubsystem;

public class RobotContainer {

  private final DriveSubsystem driveSubsystem = new DriveSubsystem();
  private final IntakeSubsystem intakeSubsytem = new IntakeSubsystem();
  private final GripperSubsystem gripperSubsystem = new GripperSubsystem();
  private final LiftSubsystem liftSubsytem = new LiftSubsystem();

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
   * edu.wpi.first.-wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    //region Intake
    controller.leftTrigger(0.2)
      .onTrue(new IntakeCommand(intakeSubsytem, gripperSubsystem, false));
    controller.rightTrigger(0.2)
      .onTrue(new IntakeCommand(intakeSubsytem, gripperSubsystem, true));
    //endregion
    
    //region Slow Mode
    controller.leftBumper()
      .onTrue(Commands.run(() -> driveSubsystem.setOutput(0.3)))
      .onFalse(Commands.run(() -> driveSubsystem.setOutput(1)));
    //endregion
  
    controller.pov(0)
    .onTrue(Commands.run(() -> liftSubsytem.liftUp()))
    .onFalse(Commands.run(() -> liftSubsytem.liftStop()));

    controller.pov(180)
    .onTrue(Commands.run(() -> liftSubsytem.liftDown()))
    .onFalse(Commands.run(() -> liftSubsytem.liftStop()));

    controller.y()
    .onTrue(Commands.run(() -> liftSubsytem.spindleUp()))
    .onFalse(Commands.run(() -> liftSubsytem.spindleStop()));

    controller.a()
    .onTrue(Commands.run(() -> liftSubsytem.spindleDown()))
    .onFalse(Commands.run(() -> liftSubsytem.spindleStop()));
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @/return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return Commands.print("No autonomous command");
  }
}