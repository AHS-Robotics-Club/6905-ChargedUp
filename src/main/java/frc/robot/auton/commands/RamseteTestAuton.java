package frc.robot.auton.commands;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.auton.ramsete.CustomRamseteCommand;
import frc.robot.auton.ramsete.CustomTrajectoryGenerator;
import frc.robot.subsystems.DriveSubsystem;

public class RamseteTestAuton extends SequentialCommandGroup {

  Trajectory exampleTraj =
    CustomTrajectoryGenerator.generateTrajectory(
      new Pose2d(0, 0, new Rotation2d(0)), // NOTE: in radians
      List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
      new Pose2d(3, 0, new Rotation2d(0))
    );

  
  public RamseteTestAuton(DriveSubsystem drive) {

    RamseteCommand ramseteCommand = new CustomRamseteCommand(exampleTraj, drive);

    addCommands(
      Commands.runOnce(() -> drive.resetOdometry(exampleTraj.getInitialPose())),
      ramseteCommand,
      Commands.runOnce(() -> drive.driveVolts(0, 0))
    );
  }
}
