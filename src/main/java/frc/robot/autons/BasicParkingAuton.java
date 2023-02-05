package frc.robot.autons;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;

public class BasicParkingAuton extends SequentialCommandGroup {

  private DriveSubsystem driveS;

  public BasicParkingAuton() {

    driveS = new DriveSubsystem();

    addCommands(
      Commands.runOnce(() -> driveS.resetOdometry(new Pose2d(0.0, 0.0, new Rotation2d(0.0)))),
      Commands.runOnce(() -> driveS.driveVolts(0.8, 0)),
      Commands.waitSeconds(1.0),
      Commands.runOnce(() -> driveS.driveVolts(0.0, 0.0))
    );

  }
  
}
