package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {

    private final CANSparkMax leftMain = new CANSparkMax(DriveConstants.LEFT_MOTOR_PORT_1, MotorType.kBrushless);
    private final CANSparkMax leftFollower = new CANSparkMax(DriveConstants.LEFT_MOTOR_PORT_2, MotorType.kBrushless);
    private final CANSparkMax rightMain = new CANSparkMax(DriveConstants.RIGHT_MOTOR_PORT_1, MotorType.kBrushless);
    private final CANSparkMax rightFollower = new CANSparkMax(DriveConstants.RIGHT_MOTOR_PORT_2, MotorType.kBrushless);

    private final RelativeEncoder leftEncoder;
    private final RelativeEncoder rightEncoder;
    private final Gyro gyro = new ADXRS450_Gyro();

    private final DifferentialDrive drive;
    private final DifferentialDriveOdometry odometry;

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

        leftEncoder = leftMain.getEncoder();
        rightEncoder = rightMain.getEncoder();
        resetEncoders();

        odometry = new DifferentialDriveOdometry(gyro.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition());
        drive = new DifferentialDrive(leftMain, rightMain);
        setToMaxOutput(); 

    }

    @Override
    public void periodic() {
        odometry.update(gyro.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition());
    }

    public void arcadeDrive(double forward, double rotation) {
        drive.arcadeDrive(forward, rotation);
    }

    public void setToMaxOutput() {
        drive.setMaxOutput(DriveConstants.MAX_OUTPUT); 
    }

    public void setToMinOutput() {
        drive.setMaxOutput(DriveConstants.MIN_OUTPUT); 
    }

    /** Auton stuff **/

    public void resetEncoders() {
        leftEncoder.setPosition(0.0);
        rightEncoder.setPosition(0.0);
    }

    public void resetGyro() {
        gyro.reset();
    }

    /**
     *  Returns heading of robot 
     * 
     * @return The robot's heading in degrees (-180 to 180)
     */
    public double getHeading() {
        return gyro.getRotation2d().getDegrees();
    }

    /**
     * Returns turn rate of robot
     * 
     * @return The turn rate of the robot in degrees per second
     */
    public double getTurnRate() {
        return -gyro.getRate();
    }

    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */
    public Pose2d getPose() {
      return odometry.getPoseMeters();
    }

     /**
    * Returns the current wheel speeds of the robot.
    *
    * @return The current wheel speeds.
    */
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
      return new DifferentialDriveWheelSpeeds(leftEncoder.getVelocity(), rightEncoder.getVelocity());
    }

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetOdometry(Pose2d pose) {
      resetEncoders();
      odometry.resetPosition(
        gyro.getRotation2d(), 
        leftEncoder.getPosition(), 
        rightEncoder.getPosition(),
        pose
      );
    }

    /**
     * Controls the left and right sides of the drive directly with voltages.
     *
     * @param leftVolts the commanded left output
     * @param rightVolts the commanded right output
     */
    public void driveVolts(double leftVolts, double rightVolts) {
      leftMain.setVoltage(leftVolts);
      rightMain.setVoltage(rightVolts);
      drive.feed();
    }

    /**
     * Gets the average distance of the two encoders
     * 
     * @return the average of the two encoder readings
     */
    public double getAverageEncoderDistance() {
        return (leftEncoder.getPosition() + rightEncoder.getPosition()) / 2.0;
    }

    public RelativeEncoder getLeftEncoder() {
        return leftEncoder;
    }

    public RelativeEncoder getRightEncoder() {
        return rightEncoder;
    }

}