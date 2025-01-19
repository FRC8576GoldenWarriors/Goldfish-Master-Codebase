// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import java.text.DecimalFormat;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
//import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {


   private SwerveModule leftFront = new SwerveModule(
    Constants.SwerveConstants.LEFT_FRONT_DRIVE_ID, 
    Constants.SwerveConstants.LEFT_FRONT_TURN_ID, 
    false,
    true, //true
    Constants.SwerveConstants.LEFT_FRONT_CANCODER_ID, 
    Constants.SwerveConstants.LEFT_FRONT_OFFSET);

  private SwerveModule rightFront = new SwerveModule(
    Constants.SwerveConstants.RIGHT_FRONT_DRIVE_ID, 
    Constants.SwerveConstants.RIGHT_FRONT_TURN_ID, 
    false, //used to be true, might have to change back - Om: 2/14/24
    true, 
    Constants.SwerveConstants.RIGHT_FRONT_CANCODER_ID, 
    Constants.SwerveConstants.RIGHT_FRONT_OFFSET);

  private SwerveModule leftBack = new SwerveModule(
    Constants.SwerveConstants.LEFT_BACK_DRIVE_ID, 
    Constants.SwerveConstants.LEFT_BACK_TURN_ID, 
    true, 
    true, 
    Constants.SwerveConstants.LEFT_BACK_CANCODER_ID, 
    Constants.SwerveConstants.LEFT_BACK_OFFSET);

    private SwerveModule rightBack = new SwerveModule(
    Constants.SwerveConstants.RIGHT_BACK_DRIVE_ID, 
    Constants.SwerveConstants.RIGHT_BACK_TURN_ID, 
    false, 
    true, 
    Constants.SwerveConstants.RIGHT_BACK_CANCODER_ID, 
    Constants.SwerveConstants.RIGHT_BACK_OFFSET);

  private SlewRateLimiter frontLimiter = new SlewRateLimiter(Constants.SwerveConstants.TELE_DRIVE_MAX_ACCELERATION);
  private SlewRateLimiter sideLimiter = new SlewRateLimiter(Constants.SwerveConstants.TELE_DRIVE_MAX_ACCELERATION);
  private SlewRateLimiter turnLimiter = new SlewRateLimiter(Constants.SwerveConstants.TELE_DRIVE_MAX_ANGULAR_ACCELERATION);

  private Pigeon2 gyro = new Pigeon2(Constants.SwerveConstants.PIGEON_ID);

  private static final Drivetrain drivetrain = new Drivetrain();

  private RobotConfig config;

  //getHeadingRotation2d()
  public SwerveDriveOdometry odometry = new SwerveDriveOdometry(Constants.SwerveConstants.DRIVE_KINEMATICS,  getHeadingRotation2d() , getModulePositions(), new Pose2d());
  private Field2d field;
  //private final StructPublisher<Pose2d> m_posePublish;
    private final StructArrayPublisher<SwerveModuleState> m_ModulePublisherIn;
    private final StructArrayPublisher<SwerveModuleState> m_ModuleStatesActual;
    private final StructPublisher<Pose2d> m_pose;
  public static Drivetrain getInstance(){
    return drivetrain;
  }

  /** Creates a new SwerveDrivetrain. */
  public Drivetrain() {
    new Thread(() -> {
      try{
        Thread.sleep(1000);
        zeroHeading();
      }
      catch(Exception e){}
    }).start();
    field = new Field2d();
    // AutoBuilder.configureHolonomic(
    //   this::getPose2d,
    //   this::resetPose2d,
    //   this::getRobotRelativeSpeeds,
    //   this::driveRobotRelative,
    //   Constants.SwerveConstants.AUTO_CONFIG,
    //   () -> isRedAlliance(),
    //   this
    // );

     // Load the RobotConfig from the GUI settings. You should probably
    // store this in your Constants file
    
    try{
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

        // test
    // AutoBuilder.configure(
    //         this::getPose2d, // Robot pose supplier
    //         this::resetPose2d, // Method to reset odometry (will be called if your auto has a starting pose)
    //         this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
    //         this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
    //         Constants.SwerveConstants.pid_controls,
    //         config, // The robot configuration
    //         () -> {
    //           // Boolean supplier that controls when the path will be mirrored for the red alliance
    //           // This will flip the path being followed to the red side of the field.
    //           // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

    //           var alliance = DriverStation.getAlliance();
    //           if (alliance.isPresent()) {
    //             return alliance.get() == DriverStation.Alliance.Red;
    //           }
    //           return false;
    //         },
    //         this // Reference to this subsystem to set requirements
    // );
  
    //original
    // Configure AutoBuilder last
    AutoBuilder.configure(
            this::getPose2d, // Robot pose supplier
            this::resetPose2d, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            Constants.SwerveConstants.pid_controls,
            config, // The robot configuration
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );
    SmartDashboard.putData("GWR Field",field);
    m_ModulePublisherIn = NetworkTableInstance.getDefault().getTable("Goldfish").getStructArrayTopic("SwerveStates/In", SwerveModuleState.struct).publish();
    m_ModuleStatesActual = NetworkTableInstance.getDefault().getTable("Goldfish").getStructArrayTopic("SwerveStates/Actual", SwerveModuleState.struct).publish();
   m_pose = NetworkTableInstance.getDefault().getTable("Goldfish").getStructTopic("Pose",Pose2d.struct).publish();
    //m_posePublish = NetworkTableInstance.getDefault().getTable("Goldfish").getStructTopic("Robot Pose", Pose2d.struct).publish();
    SmartDashboard.putData("Swerve Drive", new Sendable() {
      @Override
      public void initSendable(SendableBuilder builder){
        builder.setSmartDashboardType("SwerveDrive");

        builder.addDoubleProperty("Front Left Angle", () -> leftFront.getTurnMotorPosition(), null);
        builder.addDoubleProperty("Front Left Velocity", () -> leftFront.getDriveMotorVelocity(), null);

        builder.addDoubleProperty("Front Right Angle", () -> rightFront.getTurnMotorPosition(), null);
        builder.addDoubleProperty("Front Right Velocity", () -> rightFront.getDriveMotorVelocity(), null);

        builder.addDoubleProperty("Back Left Angle", () -> leftBack.getTurnMotorPosition(), null);
        builder.addDoubleProperty("Back Left Velocity", () -> leftBack.getDriveMotorVelocity(), null);

        builder.addDoubleProperty("Back Right Angle", () -> rightBack.getTurnMotorPosition(), null);
        builder.addDoubleProperty("Back Right Velocity", () -> rightBack.getDriveMotorVelocity(), null);

        builder.addDoubleProperty("Robot Angle", () -> (getHeading() / 180 * Math.PI), null);
      }
    });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //RobotContainer.poseEstimator.updateOdometry(getHeadingRotation2d(), getModulePositions());
    
    double yaw = gyro.getYaw().getValueAsDouble();
    SmartDashboard.putNumber("Robot Angle", getHeading());
    field.setRobotPose(getPose2d());
   // m_posePublish.set(getPose2d());
    m_ModuleStatesActual.set(getModuleStates());
    m_pose.set(odometry.getPoseMeters());
    //rates 2 is yaw (XYZ in order )
    /*SmartDashboard.putString("Angular Speed", new DecimalFormat("#.00").format((yaw/ 180)) + "pi rad/s");
    // Logger.recordOutput("Robot Angle", getHeading());
    // Logger.recordOutput("Robot Pitch", getPitch());
    // Logger.recordOutput("Robot Roll", getRoll());
    // Logger.recordOutput("Pose", getPose().toString());
    // Logger.recordOutput("Angular Speed", new DecimalFormat("#.00").format((yaw / 180)) + "pi rad/s" );

    SmartDashboard.putString("Pose", getPose2d().toString());

    //new values
    SmartDashboard.putNumber("Left Front Module Velocity", leftFront.getDriveMotorVelocity());
    SmartDashboard.putNumber("Right Front Module Velocity", rightFront.getDriveMotorVelocity());
    SmartDashboard.putNumber("Left Back Module Velocity", leftBack.getDriveMotorVelocity());
    SmartDashboard.putNumber("Right Back Module Velocity", rightBack.getDriveMotorVelocity());
    SmartDashboard.putNumber("Left Front Module abs angle", leftFront.getAbsoluteEncoderAngle());
    SmartDashboard.putNumber("Right Front Module abs angle", rightFront.getAbsoluteEncoderAngle());
    SmartDashboard.putNumber("Left Back Module abs angle", leftBack.getAbsoluteEncoderAngle());
    SmartDashboard.putNumber("Right Back Module abs angle", rightBack.getAbsoluteEncoderAngle());*/

    /*SmartDashboard.putData("Swerve Drive", new Sendable() {
      @Override
      public void initSendable(SendableBuilder builder){
        builder.setSmartDashboardType("SwerveDrive");

        builder.addDoubleProperty("Front Left Angle", () -> leftFront.getTurnMotorPosition(), null);
        builder.addDoubleProperty("Front Left Velocity", () -> leftFront.getDriveMotorVelocity(), null);

        builder.addDoubleProperty("Front Right Angle", () -> rightFront.getTurnMotorPosition(), null);
        builder.addDoubleProperty("Front Right Velocity", () -> rightFront.getDriveMotorVelocity(), null);

        builder.addDoubleProperty("Back Left Angle", () -> leftBack.getTurnMotorPosition(), null);
        builder.addDoubleProperty("Back Left Velocity", () -> leftBack.getDriveMotorVelocity(), null);

        builder.addDoubleProperty("Back Right Angle", () -> rightBack.getTurnMotorPosition(), null);
        builder.addDoubleProperty("Back Right Velocity", () -> rightBack.getDriveMotorVelocity(), null);

        builder.addDoubleProperty("Robot Angle", () -> getHeading(), null);
      }
    });// */
  

    // Logger.recordOutput("Drivetrain/Robot Angle", getHeadingRotation2d().getRadians());
    // Logger.recordOutput("Drivetrain/Pose", getPose());
    // Logger.recordOutput("Drivetrain/Angular Speed", yaw / 180);
    // Logger.recordOutput("Drivetrain/Module States", getModuleStates());

    odometry.update(getHeadingRotation2d(), getModulePositions());
  }

  public void swerveDrive(double frontSpeed, double sideSpeed, double turnSpeed, 
    boolean fieldOriented, Translation2d centerOfRotation, boolean deadband){ 
      //Drive with rotational speed control w/ joystick
    if(deadband){
      frontSpeed = Math.abs(frontSpeed) > Constants.SwerveConstants.DriverConstants.xDeadband ? frontSpeed : 0;
      sideSpeed = Math.abs(sideSpeed) > Constants.SwerveConstants.DriverConstants.yDeadband ? sideSpeed : 0;
      turnSpeed = Math.abs(turnSpeed) >Constants.SwerveConstants.DriverConstants.turnDeadband? turnSpeed : 0;
    }

    frontSpeed = frontLimiter.calculate(frontSpeed) * Constants.SwerveConstants.TELE_DRIVE_MAX_SPEED;
    sideSpeed = sideLimiter.calculate(sideSpeed) * Constants.SwerveConstants.TELE_DRIVE_MAX_SPEED;
    turnSpeed = turnLimiter.calculate(turnSpeed) * Constants.SwerveConstants.TELE_DRIVE_MAX_ANGULAR_SPEED;

    ChassisSpeeds chassisSpeeds;
    if(fieldOriented){
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(frontSpeed, sideSpeed, turnSpeed, getHeadingRotation2d());
    }
    else{
      chassisSpeeds = new ChassisSpeeds(frontSpeed, sideSpeed, turnSpeed);
    }

    SwerveModuleState[] moduleStates = Constants.SwerveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds, centerOfRotation);
     m_ModulePublisherIn.set(moduleStates);
    setModuleStates(moduleStates);
  }

  public void setAllIdleMode(boolean brake){
    if(brake){
      leftFront.setBrake(true);
      rightFront.setBrake(true);
      leftBack.setBrake(true);
      rightBack.setBrake(true);
    }
    else{
      leftFront.setBrake(false);
      rightFront.setBrake(false);
      leftBack.setBrake(false);
      rightBack.setBrake(false);
    }
  }

  public void resetAllEncoders(){
    System.out.println("resetAllEncoders()");
    leftFront.resetEncoders();
    rightFront.resetEncoders();
    leftBack.resetEncoders();
    rightBack.resetEncoders();
    odometry.resetPose(new Pose2d());
  }

  public void zeroHeading(){
    gyro.setYaw(0);
    odometry.resetRotation(gyro.getRotation2d());
  }
  public void autonReset(){
    Pose2d pose = AutoBuilder.getCurrentPose();
    double[] xy = {pose.getX(),pose.getY()};
    Pose2d calcpose = new Pose2d(xy[0], xy[1], Rotation2d.fromDegrees(180));
    odometry.resetPose(calcpose);
  }

  public void setHeading(double heading){
    gyro.setYaw(heading);
  }

  public double getHeading(){
    return (Math.IEEEremainder(gyro.getYaw().getValueAsDouble(), 360)); //clamp heading between -180 and 180
  }

  public Rotation2d getHeadingRotation2d(){
    return Rotation2d.fromDegrees(getHeading());
  }

  public void stopModules(){
    leftFront.stop();
    leftBack.stop();
    rightFront.stop();
    rightBack.stop();
  }

  public void setModuleStates(SwerveModuleState[] moduleStates){
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Constants.SwerveConstants.DRIVETRAIN_MAX_SPEED);
    leftFront.setDesiredState(moduleStates[0]);
    rightFront.setDesiredState(moduleStates[1]);
    leftBack.setDesiredState(moduleStates[2]);
    rightBack.setDesiredState(moduleStates[3]);
  }

  // public void setModuleZero(){ Not Called Anywhere
  //   leftFront.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
  //   rightFront.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
  //   leftBack.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
  //   rightBack.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
  // }

  public SwerveModuleState[] getModuleStates(){
    SwerveModuleState[] states = new SwerveModuleState[4];
    states[0] = leftFront.getState();
    states[1] = rightFront.getState();
    states[2] = leftBack.getState();
    states[3] = rightBack.getState();
    return states;
  } 

  public SwerveModulePosition[] getModulePositions(){
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    positions[0] = leftFront.getPosition();
    positions[1] = rightFront.getPosition();
    positions[2] = leftBack.getPosition();
    positions[3] = rightBack.getPosition();
    return positions;
  } 

  public Pose2d getPose2d(){
    return odometry.getPoseMeters();
  }

  public void resetPose2d(Pose2d pose){
    odometry.resetPosition(getHeadingRotation2d(), getModulePositions(), pose);
  }

  public ChassisSpeeds getRobotRelativeSpeeds(){
    return Constants.SwerveConstants.DRIVE_KINEMATICS.toChassisSpeeds(getModuleStates());
  }

  public void driveRobotRelative(ChassisSpeeds chassisSpeeds){
    SwerveModuleState[] moduleStates = Constants.SwerveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);
    setModuleStates(moduleStates);
  }

  //  public void visionDrive(AprilTagStats april,double angle){
  //    try{
  //       // Load the path you want to follow using its name in the GUI
  //       PathPlannerPath path = april.robotPath(angle);

  //       // Create a path following command using AutoBuilder. This will also trigger event markers.
  //       AutoBuilder.followPath(path);
  //   } catch (Exception e) {
  //       DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
  //   }

  // }

  public boolean isRedAlliance(){
    if (DriverStation.getAlliance().isPresent()){
      return DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
    }
    return false;
  }
}