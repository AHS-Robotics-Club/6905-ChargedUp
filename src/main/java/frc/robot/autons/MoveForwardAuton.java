package frc.robot.autons;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class MoveForwardAuton extends SequentialCommandGroup {
    
    public MoveForwardAuton(DriveSubsystem drive, IntakeSubsystem intake) {

        addCommands(
            Commands.waitSeconds(5),
            Commands.run(() -> drive.arcadeDrive(0.5, 0))
                .withTimeout(3)
        );

    }

}
