package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {

    private final CANSparkMax leftMain = new CANSparkMax(DriveConstants.LEFT_MOTOR_PORT_1, MotorType.kBrushless);
    private final CANSparkMax leftFollower = new CANSparkMax(DriveConstants.LEFT_MOTOR_PORT_2, MotorType.kBrushless);
    private final CANSparkMax rightMain = new CANSparkMax(DriveConstants.RIGHT_MOTOR_PORT_1, MotorType.kBrushless);
    private final CANSparkMax rightFollower = new CANSparkMax(DriveConstants.RIGHT_MOTOR_PORT_2, MotorType.kBrushless);

    private final DifferentialDrive drive;

    public DriveSubsystem() {

        leftMain.restoreFactoryDefaults();
        leftFollower.restoreFactoryDefaults();
        rightMain.restoreFactoryDefaults();
        rightFollower.restoreFactoryDefaults();

        leftMain.setIdleMode(IdleMode.kBrake);
        leftFollower.setIdleMode(IdleMode.kBrake);
        rightMain.setIdleMode(IdleMode.kBrake);
        rightFollower.setIdleMode(IdleMode.kBrake);

        rightMain.setInverted(true);

        leftFollower.follow(leftMain);
        rightFollower.follow(rightMain);

        drive = new DifferentialDrive(leftMain, rightMain);
        
    }

    public void arcadeDrive(double forward, double rotation) {
        drive.arcadeDrive(forward, rotation);
    }

    public void driveVolts(double leftVolts, double rightVolts) {
        leftMain.setVoltage(leftVolts);
        rightMain.setVoltage(rightVolts);
        drive.feed();
    }

    public void setOutput(double outputSpeed) {
        leftMain.set(outputSpeed);
        rightMain.set(outputSpeed);
    }

}