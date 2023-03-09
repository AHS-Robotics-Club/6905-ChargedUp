package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;

public class GripperSubsystem extends SubsystemBase{

    // TODO: Fix CAN id
    private final CANSparkMax leftMotor = new CANSparkMax(MotorConstants.LEFT_MOTOR_GRIPPER, MotorType.kBrushless);
    private final CANSparkMax rightMotor = new CANSparkMax(MotorConstants.RIGHT_MOTOR_GRIPPER, MotorType.kBrushless);

    public GripperSubsystem() {
        rightMotor.follow(leftMotor, true);
    }

    public void intake() {
        leftMotor.set(0.5);
    }

    public void stop(){
        leftMotor.stopMotor();
    }

    public void outtake() {
        leftMotor.set(-0.5);
    }
}
