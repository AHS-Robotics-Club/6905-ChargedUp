package frc.robot.auton.ramsete;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class CustomRamseteCommand extends RamseteCommand {
  
    public CustomRamseteCommand(Trajectory trajectory, DriveSubsystem drive) {
      super(
        trajectory,
        drive::getPose,
        new RamseteController(AutoConstants.RAMSETE_B, AutoConstants.RAMSETE_ZETA),
        new SimpleMotorFeedforward(
          DriveConstants.S_VOLTS,
          DriveConstants.V_VOLT_SECONDS_PER_METER,
          DriveConstants.A_VOLT_SECONDS_SQUARED_PER_METER
        ),
        DriveConstants.DRIVE_KINEMATICS,
        drive::getWheelSpeeds,
        new PIDController(DriveConstants.P_DRIVE_VEL, 0, 0),
        new PIDController(DriveConstants.P_DRIVE_VEL, 0, 0),
        drive::driveVolts,
        drive
      );
    }
}
