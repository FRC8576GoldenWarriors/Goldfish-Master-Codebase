package frc.robot;

import java.util.Arrays;
import java.util.List;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public class Constants {

    // Vision Constants
    public static class VisionConstants {
        public static class CameraTranslationConstants {
            public static final double tX = -32 * 0.01; // Camera position in meters
            public static final double tY = 0.0 * 0.01;
            public static final double tZ = 32 * 0.01;
        }

        public static class CameraRotationConstants {
            public static final double rRoll = 0.0;
            public static final double rPitch = 0.0;
            public static final double rYaw = 0.0;
        }

        public static class DistanceConstants {
            public static final double goalMeterDistance = 3.0;
            public static final double visionAngleDegrees = 0.0;
            public static final List<Integer> useableIDs = Arrays.asList(6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22);
        }

        public static class NameConstants {
            public static final String cameraName = "Arducam_OV9281_USB_Camera (1)";
            public static final String tabName = "Vision";
            public static final String publishName = "VisionPose";
        }

        public static class VisionPIDConstants {
            public static final double kPVisionTurning = 0.01;
            public static final double kPVisionMoving = 0.5;
        }
    }

    // Controller Constants
    public static final class ControllerConstants {
        public static final int driverControllerPort = 0;
        public static final int operatorControllerPort = 1;
    }

    // Swerve Drive Constants
    public static final class SwerveConstants {
        public static class DriverConstants {
            public static final double xDeadband = 0.0825;
            public static final double yDeadband = 0.0825;
            public static final double turnDeadband = 0.0825;
            public static final double xCoefficient = 2.05;
            public static final double yCoefficient = 2.05;
            public static final double turnCoefficient = 1.675;
        }

        public static final int LEFT_FRONT_DRIVE_ID = 2;
        public static final int RIGHT_FRONT_DRIVE_ID = 4;
        public static final int LEFT_BACK_DRIVE_ID = 6;
        public static final int RIGHT_BACK_DRIVE_ID = 8;

        public static final int LEFT_FRONT_TURN_ID = 1;
        public static final int RIGHT_FRONT_TURN_ID = 3;
        public static final int LEFT_BACK_TURN_ID = 5;
        public static final int RIGHT_BACK_TURN_ID = 7;

        public static final int LEFT_FRONT_CANCODER_ID = 0;
        public static final int RIGHT_FRONT_CANCODER_ID = 1;
        public static final int LEFT_BACK_CANCODER_ID = 2;
        public static final int RIGHT_BACK_CANCODER_ID = 3;

        public static final int PIGEON_ID = 0;

        public static final double LEFT_FRONT_OFFSET = 0.228027;
        public static final double RIGHT_FRONT_OFFSET = -0.099609;
        public static final double LEFT_BACK_OFFSET = -0.000244;
        public static final double RIGHT_BACK_OFFSET = -0.113525;

        public static final double WHEEL_DIAMETER = Units.inchesToMeters(4.00);
        public static final double DRIVE_MOTOR_GEAR_RATIO = 6.75;
        public static final double TURN_MOTOR_GEAR_RATIO = 150.0 / 7;
        public static final double DRIVE_MOTOR_PCONVERSION = WHEEL_DIAMETER * Math.PI / DRIVE_MOTOR_GEAR_RATIO;
        public static final double TURN_MOTOR_PCONVERSION = 2 * Math.PI / TURN_MOTOR_GEAR_RATIO;
        public static final double DRIVE_MOTOR_VCONVERSION = DRIVE_MOTOR_PCONVERSION / 60.0;
        public static final double TURN_MOTOR_VCONVERSION = TURN_MOTOR_PCONVERSION / 60.0;
        public static final double KP_TURNING = 0.575;

        public static final double DRIVETRAIN_MAX_SPEED = 5.3;
        public static final double DRIVETRAIN_MAX_ANGULAR_SPEED = 5 * Math.PI;

        public static final double TELE_DRIVE_MAX_SPEED = DRIVETRAIN_MAX_SPEED / 0.85;
        public static final double TELE_DRIVE_MAX_ANGULAR_SPEED = DRIVETRAIN_MAX_ANGULAR_SPEED / 1.75;
        public static final double TELE_DRIVE_MAX_ACCELERATION = 7.5;
        public static final double TELE_DRIVE_MAX_ANGULAR_ACCELERATION = 15;

        public static final double TRACK_WIDTH = Units.inchesToMeters(25.0);
        public static final double WHEEL_BASE = Units.inchesToMeters(21.0);
        public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
                new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),
                new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),
                new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2),
                new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2));
    }

    // Climber Constants
    public static final class ClimberConstants {
        public static final int motorID = 40; // Replace with actual motor ID
        public static final boolean motorIsInverted = false;
    }

    // Coral Roller Constants
    public static final class CoralRollerConstants {
        public static final int coralRollerID = 10;
        public static final int coralRollerDigiSensorID = 1;

        public static final boolean motorIsInverted = false;

        public static final double coralIntakeInSpeed = 0.5;
        public static final double coralIntakeOutSpeed = -0.5;
    }

    // Algae Arm Constants
    public static final class AlgaeArmConstants {
        public static final class PincherConstants {
            public static final int pincherID = 20;
            public static final int pincherDigiSensorID = 2;

            public static final boolean motorIsInverted = false;

            public static final double pincherInSpeed = 0.5;
            public static final double pincherOutSpeed = -0.5;
        }

        public static final class ArmConstants {
            public static final int armMotorID = 21;
            public static final int armEncoderDIO = 3;

            public static final boolean motorIsInverted = false;

            public static final double kS = 0.1;
            public static final double kG = 0.1;
            public static final double kV = 0.1;
            public static final double kA = 0.1;

            public static final double kP = 0.1;
            public static final double kI = 0;
            public static final double kD = 0;

            public static final double startPosition = 0;
            public static final double lowReefPosition = 0.3;
            public static final double highReefPosition = 0.45;
            public static final double transportPosition = 0.65;

            public static final double lowSoftStopPosition = -0.05;
            public static final double highSoftStopPosition = 0.75;
        }
    }

    // Ground Intake Constants
    public static final class GroundIntakeConstants {
        public static final class HardwareConstants {
            public static final int rollerMotorID = 30;
            public static final boolean rollerMotorIsInverted = false;

            public static final int pivotMotorID = 31;
            public static final boolean pivotMotorIsInverted = false;

            public static final int pivotEncoderDIO = 4;
            public static final double pivotEncoderFullRange = 1.0;
            public static final double pivotEncoderZero = 0.0;

            public static final int digitalInputDIO = 5;
        }

        public static final class ControlConstants {
            public static final double groundIntakeUpPosition = 0.0;
            public static final double groundIntakeDownPosition = 0.25;

            public static final double groundIntakeInSpeed = 0.5;
            public static final double groundIntakeOutSpeed = -0.5;
        }
    }
}
