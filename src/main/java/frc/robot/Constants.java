package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.FSLib2025.swerve.SwerveModuleConstants;

public class Constants {

    public static final class RobotConstants {
        public static final double PERIODIC_INTERVAL = 0.02; // the periodic ,in seconds
        public static final String CANBUS_NAME = "RTX10390";
        public static final int DRIVE_CONTROLLER_PORT0 = 0;
        public static final int DRIVE_CONTROLLER_PORT1 = 1;
        public static final int DRIVE_CONTROLLER_PORT2 = 2;
    }

    // !Swerve
    public static final class SwerveConstants {
        public static final double MAX_MODULE_SPEED = 2;
        public static final double MAX_ACCELERATION = 0.5;
        public static final double MAX_MODULE_ROTATIONAL_SPEED = 4;

        public static final double WHEEL_BASE = 0.583;

        public static final double MODULE_MOTOR_KP = 0.008;
        public static final double MODULE_MOTOR_KI = 0;
        public static final double MODULE_MOTOR_KD = 0.005;
        public static final double STEER_MOTOR_WINDUP = 0.0;
        public static final int STEER_MOTOR_LIMIT = 0;

        public static final double DRIVE_MOTOR_GEAR_RATIO = 6.122449;
        public static final double DRIVE_WHEEL_DIAMETERS = 4 * 0.0254; // meters
        public static final double DRIVE_WHEEL_PERIMETER = Math.PI * DRIVE_WHEEL_DIAMETERS; // meters

        public static final Translation2d[] MODULE_TRANSLATOIN_METERS = new Translation2d[] {
                new Translation2d(WHEEL_BASE / 2.0, -WHEEL_BASE / 2.0),
                new Translation2d(WHEEL_BASE / 2.0, WHEEL_BASE / 2.0),
                new Translation2d(-WHEEL_BASE / 2.0, WHEEL_BASE / 2.0),
                new Translation2d(-WHEEL_BASE / 2.0, -WHEEL_BASE / 2.0)
        };

        public static final SwerveModuleConstants RF_CONSTANTS = new SwerveModuleConstants();
        static {
            RF_CONSTANTS.DriveMotorId = 11;
            RF_CONSTANTS.SteerMotorId = 12;
            RF_CONSTANTS.CANcoderId = 1;
            RF_CONSTANTS.CANcoderOffset = 0.943408;
        }

        public static final SwerveModuleConstants LF_CONSTANTS = new SwerveModuleConstants();
        static {
            LF_CONSTANTS.DriveMotorId = 21;
            LF_CONSTANTS.SteerMotorId = 22;
            LF_CONSTANTS.CANcoderId = 2;
            LF_CONSTANTS.CANcoderOffset = -0.391846;
        }

        public static final SwerveModuleConstants LB_CONSTANTS = new SwerveModuleConstants();
        static {
            LB_CONSTANTS.DriveMotorId = 31;
            LB_CONSTANTS.SteerMotorId = 32;
            LB_CONSTANTS.CANcoderId = 3;
            LB_CONSTANTS.CANcoderOffset = 0.019174;
        }

        public static final SwerveModuleConstants RB_CONSTANTS = new SwerveModuleConstants();
        static {
            RB_CONSTANTS.DriveMotorId = 41;
            RB_CONSTANTS.SteerMotorId = 42;
            RB_CONSTANTS.CANcoderId = 4;
            RB_CONSTANTS.CANcoderOffset = 0.115967;
        }

    }

    // !Elevator
    public static final class ElevatorConstants {
        public static final int LEFT_ELEVATOR_MOTOR_ID = 2;
        public static final int RIGHT_ELEVATOR_MOTOR_ID = 1;

        public static final double ELEVATOR_SPEED = 1;

        public static final double ELEVATOR_TOLERANCE = 1.0;

        public static final double ELEVATOR_MAX_POSITION = 100;
        public static final double ELEVATOR_MIN_POSITION = -1;

        public static final double ELEVATOR_DEFAULT_HEIGHT = 2;// 2
        public static final double ELEVATOR_L2_HEIGHT = 18;// 62
        public static final double ELEVATOR_L3_HEIGHT = 62;// 62
        public static final double ELEVATOR_L4_HEIGHT = 133;// 133
        public static final double ELEVATOR_CORAL_STATION_HEIGHT = 7;// 7

        public static final double ELEVATOR_MAX_VELOCITY = 1;
        public static final double ELEVATOR_MAX_ACCELERATION = 0.1;

        public static final double CRUISE_VELOCITY = 500;
        public static final double ACCELERATION = 1000;
        public static final double ALLOWED_ERROR = 0.5;

        public static final double ELEVATOR_LKP = 0.17;
        public static final double ELEVATOR_KI = 0;
        public static final double ELEVATOR_KD = 0;
        public static final double ELEVATOR_RKP = 0.08;

        public static final double ELEVATOR_KS = 0.2;
        public static final double ELEVATOR_KG = 0.1;
        public static final double ELEVATOR_KV = 1.2;
        public static final double ELEVATOR_KA = 0.2;
    }

    public static final class CombinedControlConstants {
        // !CLIMB
        public static final int CLIMB_MOTOR_ID = 1;

        public static final double CLIMB_SPEED = 0.5;

        // !ARM
        public static final int ARM_MOTOR_ID = 3;
        public static final int ARM_CANCODER_ID = 5;

        public static final double ARM_DEFAULT_OFFSET = 0.43;// arm0.43
        public static final double ARM_HORIZON_OFFSET = 0.173;
        public static final double ARM_CORAL_STATION_OFFSET = 0.46;
        public static final double ARM_PUT_OFFSET = 0.37;
        public static final double ARM_MIN_OFFSET = 0.19;
        public static final double ARM_MAX_OFFSET = 0.46;

        public static final double ARM_TOLERANCE = 0.01;

        public static final Rotation2d ARM_DEFAULT_ANGLE = Rotation2d.fromRotations(0.38);
        public static final Rotation2d ARM_CORAL_STATION_ANGLE = Rotation2d.fromRotations(0.366);
        public static final Rotation2d ARM_HORIZON_ANGLE = Rotation2d.fromRotations(0.377);
        public static final Rotation2d ARM_MIN_ANGLE = Rotation2d.fromRotations(0.0032);
        public static final Rotation2d ARM_MAX_ANGLE = Rotation2d.fromRotations(0.366);

        public static final double ARM_SPEED = 0.1;
        public static final double ARM_BACKSPEED = 0.11;
        public static final double ARM_AUTO_SPEED = 0.4;
        public static final boolean ARM_CANCODER_REVERSED = true;

        public static final double ARM_MAX_ANGULAR_VELOCITY = 0.1;
        public static final double ARM_MAX_ARGULAR_ACCELERATION = 0.4;

        public static final double ARM_KP = 0.03;
        public static final double ARM_KI = 0;
        public static final double ARM_KD = 0.01;

        public static final double ARM_KS = 0.1;
        public static final double ARM_KG = 0.1;
        public static final double ARM_KV = 0;
        public static final double ARM_KA = 0;

        // !INTAKE
        public static final int INTAKE_MOTOR_ID = 4;
        public static final double Intake_TIME = 2;
        public static final double INTAKING_SPEED = 0.9;
        public static final double PLACE_SPEED = -1.5;
    }
}
