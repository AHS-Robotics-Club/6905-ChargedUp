package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;

public class LiftSubsystem extends SubsystemBase {

    // TODO: Fix CAN id
    private final CANSparkMax leftLiftMotor = new CANSparkMax(MotorConstants.LEFT_MOTOR_LIFT, MotorType.kBrushless);
    private final CANSparkMax rightLiftMotor = new CANSparkMax(MotorConstants.RIGHT_MOTOR_LIFT, MotorType.kBrushless);

    private final CANSparkMax leftSpindleMotor = new CANSparkMax(MotorConstants.LEFT_MOTOR_SPLINDLE, MotorType.kBrushless);
    private final CANSparkMax rightSpindleMotor = new CANSparkMax(MotorConstants.RIGHT_MOTOR_SPLINDLE, MotorType.kBrushless);

    public LiftSubsystem() {
        rightLiftMotor.follow(leftLiftMotor, true);
        leftSpindleMotor.follow(rightSpindleMotor, true);

        leftLiftMotor.setIdleMode(IdleMode.kBrake);
        rightLiftMotor.setIdleMode(IdleMode.kBrake);

        leftSpindleMotor.setIdleMode(IdleMode.kBrake);
        rightSpindleMotor.setIdleMode(IdleMode.kBrake);
    }

    public void liftUp() {
        rightLiftMotor.set(0.5);
    }

    public void liftDown() {
        rightLiftMotor.set(-0.5);
    }

    public void spindleUp() {
        leftSpindleMotor.set(0.5);
    }

    public void spindleDown() {
        leftSpindleMotor.set(-0.5);
    };
    
} 
