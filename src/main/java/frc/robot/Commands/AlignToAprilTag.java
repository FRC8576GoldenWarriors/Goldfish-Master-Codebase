package frc.robot.Commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Limelight.AprilTagStatsLimelight;

// import frc.robot.subsystems.Limelight.SpeakerAllignment;

public class AlignToAprilTag extends Command {
  // private final SpeakerAllignment speakerAllignment;
  private AprilTagStatsLimelight aprilTagStatsLimelight;
  private Drivetrain drivetrain;

  private final PIDController rotationPID;
  private final PIDController forwardPID;
  private final PIDController sidePID;

  private final double focalLength =
      Constants.VisionConstants.limelightCameraDimensions.FOCAL_LENGTH;
  private final double pixelWidth = Constants.VisionConstants.limelightCameraDimensions.PIXEL_WIDTH;
  private final double realWidth = Constants.VisionConstants.limelightCameraDimensions.REAL_WIDTH;

  private double driveOutput;
  private double detectedWidth;

  private double rotationOutput;

  private double callibrationFactor = 1.2;

  private double currentDistance;
  private double tx;
  private double goalDistance;

  private CommandXboxController c =
      new CommandXboxController(Constants.ControllerConstants.driverControllerPort);

  public AlignToAprilTag(AprilTagStatsLimelight aprilTagStatsLimelight, Drivetrain drivetrain) {
    this.aprilTagStatsLimelight = aprilTagStatsLimelight;
    this.drivetrain = drivetrain;

    rotationPID =
        new PIDController(
            Constants.VisionConstants.VisionPIDConstants.rotationkP,
            Constants.VisionConstants.VisionPIDConstants.rotationkI,
            Constants.VisionConstants.VisionPIDConstants.rotationkD);
    rotationPID.setTolerance(
        Constants.VisionConstants.limeLightDistanceConstants.ALLOWED_ANGLE_ERROR);

    forwardPID =
        new PIDController(
            Constants.VisionConstants.VisionPIDConstants.forwardkP,
            Constants.VisionConstants.VisionPIDConstants.forwardkI,
            Constants.VisionConstants.VisionPIDConstants.forwardkD);
    forwardPID.setTolerance(
        Constants.VisionConstants.limeLightDistanceConstants.ALLOWED_DISTANCE_ERROR);

    sidePID =
        new PIDController(
            Constants.VisionConstants.VisionPIDConstants.sidekP,
            Constants.VisionConstants.VisionPIDConstants.sidekI,
            Constants.VisionConstants.VisionPIDConstants.sidekD);
    sidePID.setTolerance(
        Constants.VisionConstants.limeLightDistanceConstants.ALLOWED_DISTANCE_ERROR);

    addRequirements(aprilTagStatsLimelight, drivetrain);
  }

  // @Override
  // public void initialize(){
  //     speakerAllignment.configureAliance(alliance == Alliance.Blue);
  // }

  @Override
  public void execute() {
    // drivetrain.setAutoPose(false);

    detectedWidth =
        realWidth
            * Math.sqrt(aprilTagStatsLimelight.getArea())
            / (pixelWidth * focalLength * callibrationFactor);

    tx = aprilTagStatsLimelight.getTX();
    // currentDistance = aprilTagStatsLimelight.calculateDistance(focalLength, realWidth,
    // detectedWidth) * 0.0002;
    currentDistance = aprilTagStatsLimelight.calculateDistance();
    goalDistance = Constants.VisionConstants.limeLightDistanceConstants.DESIRED_APRIL_TAG_DISTANCE;

    SmartDashboard.putNumber("Tag distance", currentDistance);

    rotationOutput = rotationPID.calculate(tx, 0);
    // if(rotationOutput<0){
    //     rotationOutput+=(-0.04);
    // }
    // else{
    //     rotationOutput+=0.04;
    // }

    driveOutput =
        (currentDistance <= goalDistance) ? 0 : forwardPID.calculate(currentDistance, goalDistance);
    double sideOutput = sidePID.calculate(goalDistance * Math.sin(Math.toRadians(tx)), 0);

    drivetrain.drive(new Translation2d(driveOutput, sideOutput), rotationOutput, false, true);
    SmartDashboard.putNumber("Vision PID Drive output", driveOutput);
    SmartDashboard.putNumber("Vision PID Rotate output", rotationOutput);
    SmartDashboard.putNumber("Vision PID Side output", sideOutput);
    SmartDashboard.putNumber("Side distance", goalDistance * Math.sin(Math.toRadians(tx)));
  }

  @Override
  public void end(boolean interrupted) {
    // drivetrain.setAutoPose(true);
    drivetrain.stopModules();
  }

  @Override
  public boolean isFinished() {
    return (aprilTagStatsLimelight.hasValidTargets()
            && rotationPID.atSetpoint()
            && forwardPID.atSetpoint())
        || ((currentDistance == 0) || (c.leftTrigger().getAsBoolean()));
  }
}
