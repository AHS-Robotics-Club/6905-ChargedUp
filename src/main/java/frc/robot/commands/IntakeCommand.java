package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ITSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends CommandBase {

    private IntakeSubsystem intake; 
    private ITSubsystem transport;

    private boolean isReversed;

    public IntakeCommand(IntakeSubsystem gripper, ITSubsystem its, boolean isReversed){
        intake = gripper;
        transport = its;

        this.isReversed = isReversed;

        addRequirements(intake);
        addRequirements(transport);
    }

    @Override
    public void initialize(){
        if (isReversed) {
            intake.outtake();
            transport.outtake();
        } else {
            intake.intake();
            transport.intake();
        }
    }

    @Override
    public void end(boolean interrupted){
        intake.stop();
        transport.stop();
    }
}
