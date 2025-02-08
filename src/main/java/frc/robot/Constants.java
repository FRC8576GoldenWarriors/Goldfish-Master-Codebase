package frc.robot;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public class Constants {
      public static final class SwerveConstants{

    public static class DriverConstants{
      public static final double xDeadband = 0.0825;
      public static final double yDeadband = 0.0825;
      public static final double turnDeadband = 0.0825;
      public static final double xCoefficient = 2.05;
      public static final double yCoefficient = 2.05;
      public static final double turnCoefficient = 1.675;
    }

    
      
    public static final int LEFT_FRONT_DRIVE_ID = 7;//2; //6
    public static final int RIGHT_FRONT_DRIVE_ID = 1;//4;
    public static final int LEFT_BACK_DRIVE_ID = 4;//6; // 5
    public static final int RIGHT_BACK_DRIVE_ID = 3;//8; // 2
  
    public static final int LEFT_FRONT_TURN_ID = 6;//1; //5
    public static final int RIGHT_FRONT_TURN_ID =8;// 3;
    public static  final int LEFT_BACK_TURN_ID = 5; 
    public static final int RIGHT_BACK_TURN_ID = 2;//7; //1
  
    public static final int LEFT_FRONT_CANCODER_ID = 3;//0;
    public static final int RIGHT_FRONT_CANCODER_ID = 4;//1;
    public static final int LEFT_BACK_CANCODER_ID = 0;//2; // 2
    public static final int RIGHT_BACK_CANCODER_ID = 1;//3;

    public static final int PIGEON_ID = 0;
    

    public static  double LEFT_FRONT_OFFSET = -0.482422;//-0.344971;//0.228027;
    public static  double RIGHT_FRONT_OFFSET = -0.482178;//-0.397217;//-0.099609;
    public static  double LEFT_BACK_OFFSET = 0.000977;//0.032959;//-0.000244;
    public static  double RIGHT_BACK_OFFSET = 0.479736;//-0.324463;//-0.113525;

    public static final double WHEEL_DIAMETER = Units.inchesToMeters(4.00); 
    public static final double DRIVE_MOTOR_GEAR_RATIO = 6.75;
    public static final double TURN_MOTOR_GEAR_RATIO = 150.0/7;
    public static final double DRIVE_MOTOR_PCONVERSION = WHEEL_DIAMETER * Math.PI / DRIVE_MOTOR_GEAR_RATIO;
    public static final double TURN_MOTOR_PCONVERSION = 2 * Math.PI / TURN_MOTOR_GEAR_RATIO;//2 * Math.PI
    public static final double DRIVE_MOTOR_VCONVERSION = DRIVE_MOTOR_PCONVERSION / 60.0;
    public static final double TURN_MOTOR_VCONVERSION = TURN_MOTOR_PCONVERSION / 60.0;
    public static final double KP_TURNING = 0.575;

    public static final double DRIVETRAIN_MAX_SPEED = 5.3;//4.0, 5.5;
    public static final double DRIVETRAIN_MAX_ANGULAR_SPEED = 5 * Math.PI; //3.5, 4.25, 5

    //Teleop constraints
    public static final double TELE_DRIVE_MAX_SPEED = DRIVETRAIN_MAX_SPEED / 0.85;
    public static final double TELE_DRIVE_MAX_ANGULAR_SPEED = DRIVETRAIN_MAX_ANGULAR_SPEED / 1.75;
    public static final double TELE_DRIVE_MAX_ACCELERATION = 7.5; //3
    public static final double TELE_DRIVE_MAX_ANGULAR_ACCELERATION = 15; //

    public static final double AUTO_KP_TTANSLATION = 1.35; //1.15
    public static final double AUTO_KP_ROTATIONAL = 0.1; //0.1

    public static final int ROTATION_CURRENT_LIMIT = 30;
    public static final int DRIVE_CURRENT_LIMIT = 45;

    public static final double TRACK_WIDTH = Units.inchesToMeters(25.0);// Y WIDTH
    public static final double WHEEL_BASE = Units.inchesToMeters(21.0); // X LENGTH
    public static final double DRIVE_BASE_RADIUS = Math.sqrt((Math.pow(TRACK_WIDTH, 2) + Math.pow(WHEEL_BASE, 2))) / 2.0;

    public static final PPHolonomicDriveController pid_controls = new PPHolonomicDriveController(
      new PIDConstants(AUTO_KP_TTANSLATION, 0, 0),
      new PIDConstants(AUTO_KP_ROTATIONAL, 0, 0));
    
    //CREATE NEW CONSTANTS FOR LENGTH AND WIDTH
    //Swerve Kinematics
    public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
      new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),
      new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),
      new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2),
      new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2)
    );

  }
    public static class IntakeConstants{
        public static final class HardwareConstants{
            public static final int pivotMotorId = 13;
            public static final int pivotAbsoluteEncoder = 1;
            public static final double pivotEncoderZero = 0;
            public static final double pivotEncoderRange = 20;
            public static final double kMaxArmVelocity = 0.5;//3.55;
            public static final double kMaxArmAcceleration = 0.1;
        }

        public static final class ControlConstants{
            //PID Constants
            public static final double kP = 0.01;
            public static final double kI = 0;
            public static final double kD = 0;

            //Feed Forward COnstants
            public static final double kS = 0;
            public static final double kV = 0.49;
            public static final double kG = 2.11;
            public static final double kA = 0.05;

        }
    }
}
