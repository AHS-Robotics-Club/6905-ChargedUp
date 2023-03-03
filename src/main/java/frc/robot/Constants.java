package frc.robot;

public class Constants {

    // TODO: Update all constants
    
    public static final class DriveConstants {

        // Left Drive
        public static final int LEFT_MOTOR_PORT_1 = 5;
        public static final int LEFT_MOTOR_PORT_2 = 6;

        // Right Drive
        public static final int RIGHT_MOTOR_PORT_1 = 7;
        public static final int RIGHT_MOTOR_PORT_2 = 8;

        // Drive motor speeds
        public static final double MAX_OUTPUT = 0.5; // 0.8
        public static final double MIN_OUTPUT = 0.2;

        // Voltage
        public static final double S_VOLTS = 0.22;
        public static final double V_VOLT_SECONDS_PER_METER = 1.98;
        public static final double A_VOLT_SECONDS_SQUARED_PER_METER = 0.2;

        // Drive velocity
        public static final double P_DRIVE_VEL = 8.5;

    }

    public static final class IOConstants {

        // Xbox Controller
        public static final int DRIVER_CONTROLLER_PORT_1 = 1;

    }

    public static final class AutoConstants {

        // Max Trajectory
        public static final double MAX_SPEED_METERS_PER_SECOND = 3;
        public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 3;
        
        // Ramsete follower values
        // TODO: Tune these, ok defaults
        public static final double RAMSETE_B = 2;
        public static final double RAMSETE_ZETA = 0.7;

    }

}
