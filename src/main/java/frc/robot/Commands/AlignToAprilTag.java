package frc.robot.Commands;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.RobotContainer;
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

  private final double focalLength = Constants.VisionConstants.LimelightConstants.FOCAL_LENGTH;
  private final double pixelWidth = Constants.VisionConstants.LimelightConstants.PIXEL_WIDTH;
  private final double realWidth = Constants.VisionConstants.LimelightConstants.REAL_WIDTH;
  private double detectedWidth; // This is only needed for the distance calc using area

  private double driveOutput;
  private double sideOutput;
  private double rotationOutput;


  private double callibrationFactor = 1.2;

  private double cameraPitch = aprilTagStatsLimelight.isBargeLimelight() 
  ? Constants.VisionConstants.LimelightConstants.BargeLimelightConstants.DimensionConstants.CAMERA_PITCH : 
  Constants.VisionConstants.LimelightConstants.ReefLimelightConstants.DimensionConstants.CAMERA_PITCH;

  private double currentDistance;
  private double tx;
  private double ty;

  private double goalDistance;

  private CommandXboxController c =
      new CommandXboxController(Constants.ControllerConstants.driverControllerPort);

  public AlignToAprilTag(
      AprilTagStatsLimelight aprilTagStatsLimelight,
      Drivetrain drivetrain) {
    this.aprilTagStatsLimelight = aprilTagStatsLimelight;
    this.drivetrain = drivetrain;

    rotationPID =
        new PIDController(
            Constants.VisionConstants.VisionPIDConstants.rotationkP,
            Constants.VisionConstants.VisionPIDConstants.rotationkI,
            Constants.VisionConstants.VisionPIDConstants.rotationkD);
    rotationPID.setTolerance(
        Constants.VisionConstants.LimelightConstants.ALLOWED_ANGLE_ERROR);

    forwardPID =
        new PIDController(
            Constants.VisionConstants.VisionPIDConstants.forwardkP,
            Constants.VisionConstants.VisionPIDConstants.forwardkI,
            Constants.VisionConstants.VisionPIDConstants.forwardkD);
    forwardPID.setTolerance(
        Constants.VisionConstants.LimelightConstants.ALLOWED_DISTANCE_ERROR);

    sidePID =
        new PIDController(
            Constants.VisionConstants.VisionPIDConstants.sidekP,
            Constants.VisionConstants.VisionPIDConstants.sidekI,
            Constants.VisionConstants.VisionPIDConstants.sidekD);
    sidePID.setTolerance(
        Constants.VisionConstants.LimelightConstants.ALLOWED_DISTANCE_ERROR);

    addRequirements(aprilTagStatsLimelight, drivetrain);
  }

  // @Override
  // public void initialize(){
  //     speakerAllignment.configureAliance(alliance == Alliance.Blue);
  // }

  private void GetCameraPriority() {
    // if()
  }

  @Override
  public void execute() {
    // drivetrain.setAutoPose(false);

    List<Integer> usableTags = aprilTagStatsLimelight.isBlueAlliance() 
    ? Constants.VisionConstants.aprilTagConstants.IDs.BLUE_TAG_IDS :
    Constants.VisionConstants.aprilTagConstants.IDs.RED_TAG_IDS; 

    if (aprilTagStatsLimelight.hasValidTargets() && usableTags.contains(aprilTagStatsLimelight.getID())) {
    
        detectedWidth =
            realWidth
                * Math.sqrt(aprilTagStatsLimelight.getArea())
                / (pixelWidth * focalLength * callibrationFactor);

        tx = aprilTagStatsLimelight.getTX();
        ty = aprilTagStatsLimelight.getTY();

        currentDistance =
            Math.abs(
                aprilTagStatsLimelight.calculateDistance(focalLength, realWidth, detectedWidth) // This is the distance from the camera to the target
                    * 0.0002);
        
        currentDistance *=
            Math.cos(Math.toRadians(ty + cameraPitch)); // This is the distance from the bot to what the target's wall (adjacent side)

        currentDistance = Math.abs(currentDistance);
        
        // currentDistance += 0.1;
        // currentDistance *= 1.8;
        // currentDistance = aprilTagStatsLimelight.calculateDistance();

        goalDistance =
            aprilTagStatsLimelight.isBargeLimelight() 
            ? Constants.VisionConstants.LimelightConstants.BargeLimelightConstants.DistanceConstants.DESIRED_APRIL_TAG_DISTANCE_BARGE : 
            Constants.VisionConstants.LimelightConstants.ReefLimelightConstants.DistanceConstants.DESIRED_APRIL_TAG_DISTANCE_REEF;   


        // if(rotationOutput<0){
        //     rotationOutput+=(-0.04);
        // }
        // else{
        //     rotationOutput+=0.04;
        // }

        driveOutput =
            (currentDistance <= goalDistance) ? 0 : forwardPID.calculate(currentDistance, goalDistance);
        

        if (aprilTagStatsLimelight.isBargeLimelight()) {
            if (aprilTagStatsLimelight.isBlueAlliance()) {
                rotationOutput = (aprilTagStatsLimelight.getID() == 14) ? 
                rotationPID.calculate(drivetrain.getHeading(), 0) : 
                rotationPID.calculate(drivetrain.getHeading(), 180);
            } else {
                rotationOutput = (aprilTagStatsLimelight.getID() == 5) ? 
                rotationPID.calculate(drivetrain.getHeading(), 0) : 
                rotationPID.calculate(drivetrain.getHeading(), 180);
            }
            sideOutput = -RobotContainer.driverController.getLeftX();
        } else {
            rotationOutput = rotationPID.calculate(tx, 0);
            sideOutput = sidePID.calculate(goalDistance * Math.tan(Math.toRadians(tx)), 0);
            SmartDashboard.putNumber("Vision PID Side output", sideOutput);
        }

        drivetrain.drive(new Translation2d(driveOutput, sideOutput), rotationOutput, false, true);
    
        SmartDashboard.putNumber("Vision PID Drive output", driveOutput);
        SmartDashboard.putNumber("Vision PID Rotate output", rotationOutput);
        SmartDashboard.putNumber("Side distance", goalDistance * Math.tan(Math.toRadians(tx)));
        SmartDashboard.putNumber("Tag distance", currentDistance);
    }
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
