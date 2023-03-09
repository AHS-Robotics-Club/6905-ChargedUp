package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;

public class IntakeSubsystem extends SubsystemBase {

    // TODO: Fix CAN id
    private final CANSparkMax leftMotor = new CANSparkMax(MotorConstants.LEFT_MOTOR_INTAKE, MotorType.kBrushless);
    private final CANSparkMax rightMotor = new CANSparkMax(MotorConstants.RIGHT_MOTOR_INTAKE, MotorType.kBrushless);

    // TODO: Create solenoid id, maybe diff constants
    private final Solenoid solenoid = new Solenoid(PneumaticsModuleType.REVPH, MotorConstants.SOLENOID_INTAKE);

    public IntakeSubsystem() {
        rightMotor.follow(leftMotor, true);
    }

    public void intake() {
        leftMotor.set(0.5);
    }

    public void stop() {
        leftMotor.stopMotor();
    }

    public void outtake() {
        leftMotor.set(-0.5);
    }

    public void dropIntake() {
        solenoid.set(true);
    }    
}
