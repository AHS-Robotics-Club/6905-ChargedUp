package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;

public class ITSubsystem extends SubsystemBase{
    private final CANSparkMax leftMotor = new CANSparkMax(MotorConstants.LEFT_MOTOR_ITS, MotorType.kBrushless);
    private final CANSparkMax rightMotor = new CANSparkMax(MotorConstants.RIGHT_MOTOR_ITS, MotorType.kBrushless);

    public ITSubsystem() {
        rightMotor.follow(leftMotor, true);
    }

    public void intake() {
        rightMotor.set(0.5);
    }

    public void stop(){
        rightMotor.stopMotor();
    }

    public void outtake() {
        rightMotor.set(-0.5);
    }
}
