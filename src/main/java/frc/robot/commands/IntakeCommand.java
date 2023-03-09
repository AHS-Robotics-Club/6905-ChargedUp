package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends CommandBase {

    private IntakeSubsystem intake; 
    private GripperSubsystem gripper;

    private boolean isReversed;

    public IntakeCommand(IntakeSubsystem intake, GripperSubsystem gripper, boolean isReversed){
        this.intake = intake;
        this.gripper = gripper;

        this.isReversed = isReversed;

        addRequirements(intake, gripper);
    }

    @Override
    public void initialize(){
        if (isReversed) {
            intake.outtake();
            gripper.outtake();
        } else {
            intake.intake();
            gripper.intake();
        }
    }

    @Override
    public void end(boolean interrupted){
        intake.stop();
        gripper.stop();
    }
}
