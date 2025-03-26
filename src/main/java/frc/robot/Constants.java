package frc.robot;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.util.Color;
import java.util.Arrays;
import java.util.List;

public class Constants {

  public static class VisionConstants {

    public static class CameraConstants {
      public static final String DRIVER_CAMERA_NAME = "DriverCamera";
      public static final String CAGE_CAMERA_NAME = "ClimbCamera";
    }

    public static class LimelightConstants {

      public static final double FOCAL_LENGTH = 4.1;
      public static final double REAL_WIDTH = 165.0;
      public static final double PIXEL_WIDTH = 320.0;

      public static final double ALLOWED_ANGLE_ERROR = 5.0;
      public static final double ALLOWED_DISTANCE_ERROR = 0.1;
      public static final double ALLOWED_STRAFE_ERROR = 1.0;


      public static class BargeLimelightConstants {

        public static final String BARGE_NETWORKTABLE_KEY = "limelight-barge";

        public static class DistanceConstants {

          public static final double DESIRED_APRIL_TAG_DISTANCE_BARGE =
              2.2; // MAKE SURE TO TUNE THIS
        }

        public static class DimensionConstants {
          public static final double CAMERA_HEIGHT = Units.inchesToMeters(28); // In inches
          public static final double CAMERA_PITCH = 29.4; // In degrees
        }
      }

      public static class ReefLimelightConstants {

        public static final String REEF_NETWORKTABLE_KEY = "limelight-reef";

        public static class DistanceConstants {

          public static final double DESIRED_APRIL_TAG_DISTANCE_REEF =
              0.6; // MAKE SURE TO TUNE THIS
        }

        public static class DimensionConstants {
          public static final double CAMERA_HEIGHT = Units.inchesToMeters(23); // In inches
          public static final double CAMERA_PITCH = -45; // In degrees
        }
      }
    }

    public static class aprilTagConstants {

      public static class IDs {

        public static final List<Integer> REEF_TAG_IDS =
            Arrays.asList(6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22);
        public static final List<Integer> BARGE_TAG_IDS = Arrays.asList(4, 5, 14, 15);
        public static final List<Integer> PROCESSOR_TAG_IDS = Arrays.asList(3, 16);
        public static final List<Integer> CORAL_STATION_TAG_IDS = Arrays.asList(1, 2, 12, 13);

        public static final List<Integer> RED_TAG_IDS =
            Arrays.asList(1, 2, 5, 6, 7, 8, 9, 10, 11, 15, 16);

        public static final List<Integer> BLUE_TAG_IDS =
            Arrays.asList(3, 4, 12, 13, 14, 17, 18, 19, 20, 21, 22);

        public static final double BARGE_HEIGHT_INCHES = 73.54;
        public static final double REEF_HEIGHT_INCHES = 12.13;

        public static final double BARGE_HEIGHT_METERS = Units.inchesToMeters(BARGE_HEIGHT_INCHES);
        public static final double REEF_HEIGHT_METERS = Units.inchesToMeters(REEF_HEIGHT_INCHES);
      }

      public static class heights {
        // In meters
        public static final double REEF_TAG_HEIGHT = 0.305;
        public static final double BARGE_TAG_HEIGHT = 1.915;
        public static final double PROCESSOR_TAG_HEIGHT = 1.305;
        public static final double CORAL_STATION_TAG_HEIGHT = 1.485;
      }
    }

    public static class VisionPIDConstants {
      public static final double rotationkP = 0.08;
      public static final double rotationkI = 0;
      public static final double rotationkD = 0.001;

      public static final double forwardkP = 1.6;
      public static final double forwardkI = 0.0;
      public static final double forwardkD = 0.001;

      public static final double strafekP = 0.1;
      public static final double strafekI = 0.0;
      public static final double strafekD = 0.001;
    }
  }

  public class ControllerConstants {
    public static final int driverControllerPort = 0;
    public static final int operatorControllerPort = 1;
  }

  public static final class SwerveConstants {

    public static class DriverConstants {
      public static final double xDeadband = 0.08;
      public static final double yDeadband = 0.08;
      public static final double turnDeadband = 0.0825;
      public static final double xCoefficient = 1.75;
      public static final double yCoefficient = 1.75;
      public static final double turnCoefficient = 1.675;
    }

    // 24K constants
    // public static final int LEFT_FRONT_DRIVE_ID = 7; // 2; //6
    // public static final int RIGHT_FRONT_DRIVE_ID = 1; // 4;
    // public static final int LEFT_BACK_DRIVE_ID = 4; // 6; // 5
    // public static final int RIGHT_BACK_DRIVE_ID = 3; // 8; // 2

    // public static final int LEFT_FRONT_TURN_ID = 6; // 1; //5
    // public static final int RIGHT_FRONT_TURN_ID = 8; // 3;
    // public static final int LEFT_BACK_TURN_ID = 5;
    // public static final int RIGHT_BACK_TURN_ID = 2; // 7; //1

    // public static final int LEFT_FRONT_CANCODER_ID = 3; // 0;
    // public static final int RIGHT_FRONT_CANCODER_ID = 4; // 1;
    // public static final int LEFT_BACK_CANCODER_ID = 0; // 2; // 2
    // public static final int RIGHT_BACK_CANCODER_ID = 1; // 3;

    // public static final int PIGEON_ID = 0;

    // public static double LEFT_FRONT_OFFSET =
    //     0.239990; // -0.240723;//0.476074;//-0.482422;//-0.344971;//0.228027;
    // public static double RIGHT_FRONT_OFFSET =
    //     0.269287; // -0.484131;//-0.482178;//-0.397217;//-0.099609;
    // public static double LEFT_BACK_OFFSET =
    //     0.230469; // 0.229248;//0.243652;//-0.036865;//0.013184;//0.032959;//-0.000244;
    // public static double RIGHT_BACK_OFFSET =
    //     -0.023682; // -0.347412;//0.466309;//0.479736;//-0.324463;//-0.113525;

    // public static boolean LEFT_FRONT_DRIVE_INVERTED = false;
    // public static boolean RIGHT_FRONT_DRIVE_INVERTED = true;
    // public static boolean LEFT_BACK_DRIVE_INVERTED = true;
    // public static boolean RIGHT_BACK_DRIVE_INVERTED = false;

    // public static boolean LEFT_FRONT_TURN_INVERTED = true;
    // public static boolean RIGHT_FRONT_TURN_INVERTED = true;
    // public static boolean LEFT_BACK_TURN_INVERTED = true;
    // public static boolean RIGHT_BACK_TURN_INVERTED = true;

    // Practice & comp things

    public static final int LEFT_FRONT_DRIVE_ID = 2;
    public static final int RIGHT_FRONT_DRIVE_ID = 4;
    public static final int LEFT_BACK_DRIVE_ID = 8;
    public static final int RIGHT_BACK_DRIVE_ID = 6;

    public static final int LEFT_FRONT_TURN_ID = 1;
    public static final int RIGHT_FRONT_TURN_ID = 3;
    public static final int LEFT_BACK_TURN_ID = 7;
    public static final int RIGHT_BACK_TURN_ID = 5;

    public static final int LEFT_FRONT_CANCODER_ID = 1;
    public static final int RIGHT_FRONT_CANCODER_ID = 2;
    public static final int LEFT_BACK_CANCODER_ID = 4;
    public static final int RIGHT_BACK_CANCODER_ID = 3;

    public static final int PIGEON_ID = 0;

    public static double LEFT_FRONT_OFFSET = 0.330322;
    public static double RIGHT_FRONT_OFFSET = 0.181885;
    public static double LEFT_BACK_OFFSET = -0.013428;
    public static double RIGHT_BACK_OFFSET = 0.178955;

    public static boolean LEFT_FRONT_DRIVE_INVERTED = true;
    public static boolean RIGHT_FRONT_DRIVE_INVERTED = true;
    public static boolean RIGHT_BACK_DRIVE_INVERTED = false;
    public static boolean LEFT_BACK_DRIVE_INVERTED = false;

    public static boolean LEFT_FRONT_TURN_INVERTED = true;
    public static boolean RIGHT_FRONT_TURN_INVERTED = true;
    public static boolean RIGHT_BACK_TURN_INVERTED = true;
    public static boolean LEFT_BACK_TURN_INVERTED = true;

    public static double PRACTICE_LEFT_FRONT_OFFSET = 0.377930;
    public static double PRACTICE_RIGHT_FRONT_OFFSET = 0.094727;
    public static double PRACTICE_LEFT_BACK_OFFSET = 0.161377;
    public static double PRACTICE_RIGHT_BACK_OFFSET = 0.274658;

    public static boolean PRACTICE_LEFT_FRONT_DRIVE_INVERTED = true;
    public static boolean PRACTICE_RIGHT_FRONT_DRIVE_INVERTED = false;
    public static boolean PRACTICE_RIGHT_BACK_DRIVE_INVERTED = false;
    public static boolean PRACTICE_LEFT_BACK_DRIVE_INVERTED = true;

    public static boolean PRACTICE_LEFT_FRONT_TURN_INVERTED = true;
    public static boolean PRACTICE_RIGHT_FRONT_TURN_INVERTED = true;
    public static boolean PRACTICE_RIGHT_BACK_TURN_INVERTED = true;
    public static boolean PRACTICE_LEFT_BACK_TURN_INVERTED = true;

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

    public static final double AUTO_KP_TRANSLATION = 1.5; // 1.15
    public static final double AUTO_KP_ROTATIONAL = 5.5; // 0.07; // 0.1

    public static final int ROTATION_CURRENT_LIMIT = 30;
    public static final int DRIVE_CURRENT_LIMIT = 45;

    public static final double TRACK_WIDTH = Units.inchesToMeters(25.0); // Y WIDTH
    public static final double WHEEL_BASE = Units.inchesToMeters(25.0); // X LENGTH
    public static final double DRIVE_BASE_RADIUS =
        Math.sqrt((Math.pow(TRACK_WIDTH, 2) + Math.pow(WHEEL_BASE, 2))) / 2.0;
    // kevin pfeffer was here
    public static final PPHolonomicDriveController pid_controls =
        new PPHolonomicDriveController(
            new PIDConstants(AUTO_KP_TRANSLATION, 0, 0.1),
            new PIDConstants(AUTO_KP_ROTATIONAL, 0, 0.001));

    // CREATE NEW CONSTANTS FOR LENGTH AND WIDTH
    // Swerve Kinematics
    public static final SwerveDriveKinematics DRIVE_KINEMATICS =
        new SwerveDriveKinematics(
            new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),
            new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),
            new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2),
            new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2));
  }

  public static final class CoralRollerConstants {
    public static final class HardwareConstants {
      public static final int coralRollerID = 10;
      public static final int coralRollerDigiSensorID = 1;

      public static final boolean motorIsInverted = false;
    }

    public static final class ControlConstants {
      public static final double coralIntakeInSpeed = 0.5;
      public static final double coralIntakeOutSpeed = -0.5;
    }
  }

  public static class EndEffectorConstants {
    public static final class HardwareConstants {
      public static final int pincherID = 20;
      public static final int coralLeftDigiSensorID = 1;
      public static final int algaeDigiSensorID = 2;



      public static final boolean motorIsInverted = true;
    }

    public static final class ControlConstants {
      public static final double pincherAlgaeSpeed = -1.0; // -0.65

      public static final double pincherCoralInSpeed = 1.00;
      public static final double pincherCoralOutSpeed = -1.00;

      public static final double pincherCoralL3OutSpeed = -0.6;

    }
  }

  public static final class ArmConstants {
    public static final class HardwareConstants {

      public static final int armMotorID = 21;
      public static final int armEncoderDIO = 3;
    }

    public static final class ControlConstants {

      public static final boolean motorIsInverted = true;

      public static final double kS = 0.015; 
      public static final double kG = 0.24;
      public static final double kV = 0.0505; 
      public static final double kA = 0;

      public static final double kP = 15.0;//30.0;
      public static final double kI = 0.0;
      public static final double kD = 0.0;

      public static final double storedPosition = 0.03;

      public static final double A1Position = 0.31;
      public static final double A2Position = 0.425;
      public static final double LolipopPosition = 0.224;

      public static final double coralStationPosition = 0.665;
      public static final double L1Position = 0.14;//0.745;
      public static final double L2Position = 0.20;
      public static final double L3Position = 0.30;
      
      public static final double handoffPosition = 0.735;

      public static final double lowSoftStopPositon = 0.0;
      public static final double highSoftStopPosition = 0.75;

      public static final double armEncoderOffset = -0.441;
      public static final boolean armEncoderIsInverted = true;
    }
  }

  public static final class GroundIntakeConstants {

    public static class HardwareConstants {

      public static final int rollerMotorID = 30;
      public static final boolean rollerMotorIsInverted = false;

      public static final int pivotMotorID = 31;
      public static final boolean pivotMotorIsInverted = true;

      public static final int pivotEncoderDIO = 4;

      public static final int digitalInputDIO = 5;
    }

    public static class ControlConstants {
      public static final double groundIntakeUpPosition = 0.015;//0.01;
      public static final double groundIntakeDownPosition = 0.15;

      public static final double algaeInSpeed = -1.0;
      public static final double algaeOutSpeed = 1.0;

      public static final double coralDropSpeed = -0.2;

      public static final double pivotEncoderFullRange = 1.0;
      public static final double pivotEncoderZero = -0.85; 
      public static final boolean pivotEncoderIsInverted = true;

      public static final double kP = 28.0;//16; // 6.5;
      public static final double kI = 0;
      public static final double kD = 0.8;

      public static final double kS = 0.0;//0.003;
      public static final double kA = 0;
      public static final double kG = 0.31;//0.14;
      public static final double kV = 0.028;//0.038;

      public static final double rollerIdlekS = -0.1;

      public static final double coastZone = 0.5;
    }
  }

  public static final class ShintakeConstants {
    public static class HardwareConstants {

      public static final int rollerMotorLowID = 32; 
      public static final boolean rollerMotorLowIsInverted = true;

      public static final int rollerMotorHighID = 33;
      public static final boolean rollerMotorHighIsInverted = false;

      public static final double pivotEncoderFullRange = 1.0;
      public static final double pivotEncoderZero = 0.0;

      public static final int lowerRollerDigitalInputDIO = 8;
    }

    public static class ControlConstants {
      public static final double lowerRollerShootRPM = 4200.0; //not used rn
      public static final double upperRollerShootRPM = 4200.0;
    }
  }

  public static final class ClimberConstants {
    public static final class HardwareConstants {
      public static final int motorID = 40;
      public static final boolean motorIsInverted = true;
      public static final int climberEncoderDIO = 6;
      public static final boolean climberEncoderIsInverted = true;
    }

    public static final class ControlConstants {
      public static final double climbUpSpeed = 1.0;
      public static final double climbDownSpeed = -1.0;

      public static final double climberUpPosition = 0.137;

      public static final double climberEncoderOffset = -0.03;
    }
  }

  public static final class LEDConstants {
    public static final class HardwareConstants {
      public static final int kLEDPort = 1;
      public static final int kLEDLength = 25;
    }

    public static final class PatternConfig {
      // climbing
      // rainbowScroll() in LEDStrip
      // Drivetrain posed and shooter rpm ready
      public static final LEDPattern kLEDPosedSolid = LEDPattern.solid(new Color(0, 255, 0));

      // Drivetrain posing or Shooter revving
      public static final LEDPattern kLEDPosingBlink = LEDPattern.solid(Color.kYellow);
      public static final double kLEDPosingBlinkSpeed = 0.075;

      // VISION searching for target
      public static final LEDPattern kLEDVisionSearchingBlink = LEDPattern.solid(Color.kWhite);
      public static final double kLEDVisionSearchingBlinkSpeed = 0.075;

      // lg ground intake
      public static final LEDPattern kLEDAlgaeGroundBreathe =
          LEDPattern.solid(new Color(0, 255, 65));
      public static final double kLEDAlgaeGroundBreatheSpeed = 2;

      // lg arm intake
      public static final LEDPattern kLEDAlgaePincherBlink =
          LEDPattern.solid(new Color(0, 255, 65));
      public static final double kLEDAlgaePincherBlinkSpeed = 0.075;

      // coral photoeletric detects coral
      public static final LEDPattern kLEDCoralDetectedBreathe = LEDPattern.solid(Color.kPurple);
      public static final double kLEDCoralDetectedBreatheSpeed = 2;

      // coral intake running
      public static final LEDPattern kLEDCoralIntakeBlink = LEDPattern.solid(Color.kPurple);
      public static final double kLEDCoralIntakeBlinkSpeed = 0.075;

      // robot idle (no status) - breathing pattern
      public static final LEDPattern kLEDNoStatusBreathe =
          LEDPattern.gradient(
              GradientType.kContinuous, new Color(255, 130, 0), new Color(200, 80, 10));
      // LEDPattern.gradient(
      //     GradientType.kContinuous, new Color(255, 200, 0), new Color(255, 255, 200));
      public static final double kLEDNoStatusBreatheSpeed = 25;

      // robot disabled - scrolling pattern
      public static final LEDPattern kLEDDisabledScroll =
          LEDPattern.solid(Color.kRed);

      public static final double kLEDDisabledScrollSpeed = 2;
      // test
      public static final double kLEDRainbowScrollSize = 0.5; // 0.0 - 1.0
      public static final int kLEDRainbowScrollSpeed = 150;
      public static final LEDPattern kLEDProgressGradientPattern =
          LEDPattern.gradient(GradientType.kDiscontinuous, Color.kRed, Color.kBlue);
      // public static final double kLEDBreatheSpeedSlow = 2;

      public static final LEDPattern kshooterNotReady = LEDPattern.solid(Color.kYellow);
      public static final double kshooterNotReadyBlinkSpeed = 0.075;

      public static final LEDPattern kShooterIsReady =
          LEDPattern.solid(
              new Color(
                  13, 225,
                  13)); // LEDPattern.gradient(GradientType.kContinuous, new Color(13, 225, 13), new
      // Color(129, 229, 129));
      public static final double kShooterIsReadyBlinkSpeed = 2.0;

      public static final LEDPattern kAprilTags = LEDPattern.solid(Color.kWhite);
      public static final double kAprilTagBlinkSpeed = 0.075;
    }
  }
}
