package frc.robot;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import java.util.Arrays;
import java.util.List;

public class Constants {

  public static class VisionConstants {
    public static class cameraTranslationConstants {
      // translation of camera in meters (change when camera has been mounted on robot)
      public static final double tX = -32 * 0.01;
      public static final double tY = 0.0 * 0.01;
      public static final double tZ = 32 * 0.01;
    }

    public static class cameraRotationConstants {
      // rotation of camera (change when camera has been mounted on robot)
      public static final double rRoll = 0.0;
      public static final double rPitch = 0.0;
      public static final double rYaw = 0.0;
    }

    public static class distanceConstants {
      public static final double goalMeterDistance = 3.0;
      public static final double visionAngleDegrees = 0.0;
      public static final List<Integer> useableIDs =
          Arrays.asList(6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22);
    }

    public static class nameConstants {
      public static final String cameraName = "Arducam_OV9281_USB_Camera (1)";
      public static final String tabName = "Vision";
      public static final String publishName = "VisionPose";
    }

    public static class VisionPIDConstants {
      public static final double kPVisionTurning = 0.01;
      public static final double kPVisionMoving = 0.5;
    }

    public static final boolean driverMode = true;
  }

  public class ControllerConstants {
    public static final int driverControllerPort = 0;
    public static final int operatorControllerPort = 1;
  }

  public static final class SwerveConstants {

    public static class DriverConstants {
      public static final double xDeadband = 0.0825;
      public static final double yDeadband = 0.0825;
      public static final double turnDeadband = 0.0825;
      public static final double xCoefficient = 2.05;
      public static final double yCoefficient = 2.05;
      public static final double turnCoefficient = 1.675;
    }

    public static class PoseConfig {
      // Increase these numbers to trust your model's state estimates less.
      public static final double kPositionStdDevX = 0.1;
      public static final double kPositionStdDevY = 0.1;
      public static final double kPositionStdDevTheta = 10;

      // Increase these numbers to trust global measurements from vision less.
      public static final double kVisionStdDevX = 2;
      public static final double kVisionStdDevY = 2;
      public static final double kVisionStdDevTheta = 1;
    }

    public static final int LEFT_FRONT_DRIVE_ID = 7; // 2; //6
    public static final int RIGHT_FRONT_DRIVE_ID = 1; // 4;
    public static final int LEFT_BACK_DRIVE_ID = 4; // 6; // 5
    public static final int RIGHT_BACK_DRIVE_ID = 3; // 8; // 2

    public static final int LEFT_FRONT_TURN_ID = 6; // 1; //5
    public static final int RIGHT_FRONT_TURN_ID = 8; // 3;
    public static final int LEFT_BACK_TURN_ID = 5;
    public static final int RIGHT_BACK_TURN_ID = 2; // 7; //1

    public static final int LEFT_FRONT_CANCODER_ID = 3; // 0;
    public static final int RIGHT_FRONT_CANCODER_ID = 4; // 1;
    public static final int LEFT_BACK_CANCODER_ID = 0; // 2; // 2
    public static final int RIGHT_BACK_CANCODER_ID = 1; // 3;

    public static final int PIGEON_ID = 0;

    public static double LEFT_FRONT_OFFSET = 0.499023; // -0.482422; // -0.344971;//0.228027;
    public static double RIGHT_FRONT_OFFSET = 0.011475; // -0.482178; // -0.397217;//-0.099609;
    public static double LEFT_BACK_OFFSET =
        -0.000244; // 0.478027;//0.000977; // 0.032959;//-0.000244;
    public static double RIGHT_BACK_OFFSET = -0.024902; // 0.479736; // -0.324463;//-0.113525;

    public static final double WHEEL_DIAMETER = Units.inchesToMeters(4.00);
    public static final double DRIVE_MOTOR_GEAR_RATIO = 6.75;
    public static final double TURN_MOTOR_GEAR_RATIO = 150.0 / 7;
    public static final double DRIVE_MOTOR_PCONVERSION =
        WHEEL_DIAMETER * Math.PI / DRIVE_MOTOR_GEAR_RATIO;
    public static final double TURN_MOTOR_PCONVERSION =
        2 * Math.PI / TURN_MOTOR_GEAR_RATIO; // 2 * Math.PI
    public static final double DRIVE_MOTOR_VCONVERSION = DRIVE_MOTOR_PCONVERSION / 60.0;
    public static final double TURN_MOTOR_VCONVERSION = TURN_MOTOR_PCONVERSION / 60.0;
    public static final double KP_TURNING = 0.575;

    public static final double DRIVETRAIN_MAX_SPEED = 5.3; // 4.0, 5.5;
    public static final double DRIVETRAIN_MAX_ANGULAR_SPEED = 5 * Math.PI; // 3.5, 4.25, 5

    // Teleop constraints
    public static final double TELE_DRIVE_MAX_SPEED = DRIVETRAIN_MAX_SPEED / 0.85;
    public static final double TELE_DRIVE_MAX_ANGULAR_SPEED = DRIVETRAIN_MAX_ANGULAR_SPEED / 1.75;
    public static final double TELE_DRIVE_MAX_ACCELERATION = 7.5; // 3
    public static final double TELE_DRIVE_MAX_ANGULAR_ACCELERATION = 15; //

    public static final double AUTO_KP_TTANSLATION = 1.35; // 1.15
    public static final double AUTO_KP_ROTATIONAL = 0.35; // 0.1

    public static final int ROTATION_CURRENT_LIMIT = 30;
    public static final int DRIVE_CURRENT_LIMIT = 45;

    public static final double TRACK_WIDTH = Units.inchesToMeters(29.0); // Y WIDTH
    public static final double WHEEL_BASE = Units.inchesToMeters(29.0); // X LENGTH
    public static final double DRIVE_BASE_RADIUS =
        Math.sqrt((Math.pow(TRACK_WIDTH, 2) + Math.pow(WHEEL_BASE, 2))) / 2.0;

    public static final PPHolonomicDriveController pid_controls =
        new PPHolonomicDriveController(
            new PIDConstants(AUTO_KP_TTANSLATION, 0, 0),
            new PIDConstants(AUTO_KP_ROTATIONAL, 0.32, 0.01));//changed I and D terms

    // CREATE NEW CONSTANTS FOR LENGTH AND WIDTH
    // Swerve Kinematics
    public static final SwerveDriveKinematics DRIVE_KINEMATICS =
        new SwerveDriveKinematics(
            new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),
            new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),
            new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2),
            new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2));
  }

  public static final class CoralRollerConstants {}

  public static final class AlgaeArmConstants {}

  public static final class AlgaeShintakeConstants {}

  public static final class ClimberConstants {}
}
