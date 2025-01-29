// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Shintake;
import frc.robot.Subsystems.Limelight.AprilTagStatsLimelight;

public class ShootWithVision extends Command {

  private final Shintake shintake;
  private final Drivetrain drivetrain;


  private AprilTagStatsLimelight aprilTagStatsLimelight;

  private final PIDController rotationPID;
  private final PIDController forwardPID;
  private final PIDController sidePID;


  private final double focalLength = Constants.VisionConstants.limelightCameraDimensions.FOCAL_LENGTH;
  private final double pixelWidth = Constants.VisionConstants.limelightCameraDimensions.PIXEL_WIDTH;
  private final double realWidth = Constants.VisionConstants.limelightCameraDimensions.REAL_WIDTH;

  private double driveOutput;
  private double detectedWidth;
  

  private double rotationOutput;
  
  private double callibrationFactor = 1.2;

  private double currentDistance;
  private double tx;
  private double goalDistance;

  public ShootWithVision(Drivetrain drivetrain, Shintake shintake) {
    this.drivetrain = drivetrain;
    this.shintake = shintake;



    this.aprilTagStatsLimelight = aprilTagStatsLimelight;

    rotationPID = new PIDController(0.08, 0.008, 0.001);
    rotationPID.setTolerance(Constants.VisionConstants.limeLightDistanceConstants.ALLOWED_ANGLE_ERROR);

    forwardPID = new PIDController(1.5, 0.001, 0.001);
    forwardPID.setTolerance(Constants.VisionConstants.limeLightDistanceConstants.ALLOWED_DISTANCE_ERROR);

    sidePID = new PIDController(1.5, 0.001, 0.001);
    sidePID.setTolerance(Constants.VisionConstants.limeLightDistanceConstants.ALLOWED_DISTANCE_ERROR);



    addRequirements(drivetrain,shintake, aprilTagStatsLimelight);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    //vision values



    //Projectile values
    double time = Math.sqrt( (2 * Constants.ShooterConstants.accelerationY) / Constants.ShooterConstants.gravity );
    double changeX = 0;
    double initialVelocityX = (changeX - (0.5 * Constants.ShooterConstants.accelerationX * Math.pow(time,2) ) ) / time;
    double initialVelocityY = (Constants.ShooterConstants.changeY - (0.5 * Constants.ShooterConstants.gravity * Math.pow(time,2) ) ) / time;
    double resultedVelocity = Math.sqrt( Math.pow(initialVelocityX, 2) + Math.pow(initialVelocityY, 2) );
    double velocityDrivetrain = drivetrain.getRobotRelativeSpeeds().vxMetersPerSecond;;
    double velocityWheels = resultedVelocity - velocityDrivetrain; // Vi - Vdrivetrain

  


    double desiredRPM = velocityWheels * 60 / (Math.PI * Constants.SwerveConstants.WHEEL_DIAMETER);
    double voltage = shintake.calculateMotorVoltage(desiredRPM);
    shintake.setRollersVoltage(voltage);
  }

  @Override
  public void end(boolean interrupted) {
    shintake.setLowerRollerSpeed(0);
    shintake.setUpperRollerSpeed(0);
  }

  @Override
  public boolean isFinished() {
    return !RobotContainer.m_groundIntake.getDigitalInput().get();
  }
}
