package frc.robot.Commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Limelight.AprilTagStatsLimelight;
import java.util.List;
import org.littletonrobotics.junction.Logger;

// import frc.robot.subsystems.Limelight.SpeakerAllignment;

public class AlignToReef extends Command {
  // private final SpeakerAllignment speakerAllignment;
  private AprilTagStatsLimelight aprilTagStatsLimelight;
  private Drivetrain drivetrain;

  private final PIDController rotationPID;
  private final PIDController forwardPID;
  private final PIDController strafePID;

  private final double focalLength = Constants.VisionConstants.LimelightConstants.FOCAL_LENGTH;
  private final double pixelWidth = Constants.VisionConstants.LimelightConstants.PIXEL_WIDTH;
  private final double realWidth = Constants.VisionConstants.LimelightConstants.REAL_WIDTH;
  private double detectedWidth; // This is only needed for the distance calc using area

  private double driveOutput;
  private double sideOutput;
  private double rotationOutput;

  private double callibrationFactor = 1.2;

  private double cameraPitch = -45;

  private double currentDistance;
  private double tx;
  private double ty;

  private double goalDistance;

  public AlignToReef(AprilTagStatsLimelight aprilTagStatsLimelight, Drivetrain drivetrain) {
    this.aprilTagStatsLimelight = aprilTagStatsLimelight;
    this.drivetrain = drivetrain;

    rotationPID =
        new PIDController(
            Constants.VisionConstants.VisionPIDConstants.rotationkP,
            Constants.VisionConstants.VisionPIDConstants.rotationkI,
            Constants.VisionConstants.VisionPIDConstants.rotationkD);
    rotationPID.setTolerance(Constants.VisionConstants.LimelightConstants.ALLOWED_ANGLE_ERROR);

    forwardPID =
        new PIDController(
            Constants.VisionConstants.VisionPIDConstants.forwardkP,
            Constants.VisionConstants.VisionPIDConstants.forwardkI,
            Constants.VisionConstants.VisionPIDConstants.forwardkD);
    forwardPID.setTolerance(Constants.VisionConstants.LimelightConstants.ALLOWED_DISTANCE_ERROR);
    strafePID =
        new PIDController(
            Constants.VisionConstants.VisionPIDConstants.strafekP,
            Constants.VisionConstants.VisionPIDConstants.strafekI,
            Constants.VisionConstants.VisionPIDConstants.strafekD);
    strafePID.setTolerance(Constants.VisionConstants.LimelightConstants.ALLOWED_STRAFE_ERROR);
    // rotationPID =
    //     new ProfiledPIDController(
    //         Constants.VisionConstants.VisionPIDConstants.rotationkP,
    //         Constants.VisionConstants.VisionPIDConstants.rotationkI,
    //         Constants.VisionConstants.VisionPIDConstants.rotationkD,
    //         new Constraints(Constants.SwerveConstants.TELE_DRIVE_MAX_ANGULAR_SPEED,
    // Constants.SwerveConstants.TELE_DRIVE_MAX_ANGULAR_ACCELERATION));
    // rotationPID.setTolerance(Constants.VisionConstants.LimelightConstants.ALLOWED_ANGLE_ERROR);

    // forwardPID =
    //     new ProfiledPIDController(
    //         Constants.VisionConstants.VisionPIDConstants.forwardkP,
    //         Constants.VisionConstants.VisionPIDConstants.forwardkI,
    //         Constants.VisionConstants.VisionPIDConstants.forwardkD,
    //         new Constraints(Constants.SwerveConstants.TELE_DRIVE_MAX_ANGULAR_SPEED,
    // Constants.SwerveConstants.TELE_DRIVE_MAX_ANGULAR_ACCELERATION));
    // forwardPID.setTolerance(Constants.VisionConstants.LimelightConstants.ALLOWED_DISTANCE_ERROR);

    addRequirements(aprilTagStatsLimelight, drivetrain);
  }

  @Override
  public void execute() {

    List<Integer> usableTags =
        aprilTagStatsLimelight.isBlueAlliance()
            ? Constants.VisionConstants.aprilTagConstants.IDs.BLUE_TAG_IDS
            : Constants.VisionConstants.aprilTagConstants.IDs.RED_TAG_IDS;
    SmartDashboard.putBoolean(
        "Can align",
        aprilTagStatsLimelight.hasValidTargets()
            && usableTags.contains(aprilTagStatsLimelight.getID()));
    SmartDashboard.putNumber("Tag Distance", currentDistance);
    Logger.recordOutput(
        "Limelight/Can align",
        aprilTagStatsLimelight.hasValidTargets()
            && usableTags.contains(aprilTagStatsLimelight.getID()));

    // if tag detected
    if (aprilTagStatsLimelight.hasValidTargets()
        && usableTags.contains(aprilTagStatsLimelight.getID())) {

      aprilTagStatsLimelight.setTagDetected(true);

      detectedWidth =
          realWidth
              * Math.sqrt(aprilTagStatsLimelight.getArea())
              / (pixelWidth * focalLength * callibrationFactor);

      tx = aprilTagStatsLimelight.getTX();
      ty = aprilTagStatsLimelight.getTY();

      currentDistance =
          Math.abs(
              aprilTagStatsLimelight.calculateDistance(
                      focalLength,
                      realWidth,
                      detectedWidth) // This is the distance from the camera to the target
                  * 0.0002);

      currentDistance *=
          Math.cos(
              Math.toRadians(
                  ty + cameraPitch)); // This is the distance from the bot to what the target's
      // wall (adjacent side)

      currentDistance = Math.abs(currentDistance);

      // currentDistance += 0.1;
      // currentDistance *= 1.8;
      // currentDistance = aprilTagStatsLimelight.calculateDistance();

      SmartDashboard.putBoolean("Is Running", true);
      Logger.recordOutput("Limelight/Is Running", true);

      goalDistance = Constants.VisionConstants.LimelightConstants.ReefLimelightConstants.DistanceConstants.DESIRED_APRIL_TAG_DISTANCE_REEF;

      // if(rotationOutput<0){
      //     rotationOutput+=(-0.04);
      // }
      // else{
      //     rotationOutput+=0.04;
      // }

      driveOutput = forwardPID.calculate(currentDistance, goalDistance);
      // (currentDistance <= goalDistance)
      //     ? 0
      //     : forwardPID.calculate(currentDistance, goalDistance);
      sideOutput = -RobotContainer.driverController.getLeftX() * 5.5;

        // blue

          // reef alignment
          if (aprilTagStatsLimelight.getID() == 18||aprilTagStatsLimelight.getID() == 7) {
            // rotationOutput = rotationPID.calculate(drivetrain.getHeading(), 0);
            // sideOutput = strafePID.calculate(ty, 0);
            rotationOutput = rotationPID.calculate(drivetrain.getHeading(), drivetrain.getHeading()>0?180:-180);
            sideOutput = strafePID.calculate(tx, 0);
          } 
          else if (aprilTagStatsLimelight.getID() == 17||aprilTagStatsLimelight.getID() == 6) {
            rotationOutput = rotationPID.calculate(drivetrain.getHeading(), -129);
            sideOutput = strafePID.calculate(tx, 0);
          } 
          else if (aprilTagStatsLimelight.getID() == 22||aprilTagStatsLimelight.getID() == 11) {
            rotationOutput = rotationPID.calculate(drivetrain.getHeading(), -70);
            sideOutput = strafePID.calculate(tx, 0);
          } 
          else if (aprilTagStatsLimelight.getID() == 21||aprilTagStatsLimelight.getID() == 10) {
            rotationOutput = rotationPID.calculate(drivetrain.getHeading(), 0);
            sideOutput = strafePID.calculate(tx, 0);
          } 
          else if (aprilTagStatsLimelight.getID() == 20||aprilTagStatsLimelight.getID() == 9) {
            rotationOutput = rotationPID.calculate(drivetrain.getHeading(), 50);
            sideOutput = strafePID.calculate(tx, 0);
          }
           else if (aprilTagStatsLimelight.getID() == 19||aprilTagStatsLimelight.getID() == 8) {
            rotationOutput = rotationPID.calculate(drivetrain.getHeading(), 105);
            sideOutput = strafePID.calculate(tx, 0);
          } 
    
        

          if (aprilTagStatsLimelight.getID() == 5) {
            rotationOutput = rotationPID.calculate(drivetrain.getHeading(), 24);
            sideOutput = strafePID.calculate(ty, 0);
          } 
          drivetrain.drive(new Translation2d(driveOutput, -sideOutput), rotationOutput, false, true);
        }
      

        //drivetrain.drive(new Translation2d(-driveOutput, -sideOutput), rotationOutput, false, true);

        if (rotationPID.getPositionError()
                < Constants.VisionConstants.LimelightConstants.ALLOWED_ANGLE_ERROR * 1.05
            && forwardPID.getPositionError()
                < Constants.VisionConstants.LimelightConstants.ALLOWED_DISTANCE_ERROR * 1.05) {
        aprilTagStatsLimelight.setTagReached(true);
        } else {
        aprilTagStatsLimelight.setTagReached(false);
        }

    SmartDashboard.putNumber("Vision PID Drive output", driveOutput);
    SmartDashboard.putNumber("Vision PID Rotate output", rotationOutput);
    SmartDashboard.putNumber("Abs rotation", Math.abs(drivetrain.getHeading()));
    SmartDashboard.putNumber("Side distance", goalDistance * Math.tan(Math.toRadians(tx)));
    SmartDashboard.putNumber("Tag distance", currentDistance);

    Logger.recordOutput("Limelight/Vision PID Drive output", driveOutput);
    Logger.recordOutput("Limelight/Vision PID Rotate output", rotationOutput);
    Logger.recordOutput("Limelight/Abs rotation", Math.abs(drivetrain.getHeading()));
    Logger.recordOutput("Limelight/Side distance", goalDistance * Math.tan(Math.toRadians(tx)));
    Logger.recordOutput("Limelight/Tag distance", currentDistance);
  }

  // tag not detected

  @Override
  public void end(boolean interrupted) {
    // drivetrain.setAutoPose(true);
    // drivetrain.setAutoPosed(false);
    drivetrain.drive(new Translation2d(0, 0), 0, false, true);
    // drivetrain.stopModules();
    aprilTagStatsLimelight.setTagReached(false);
    aprilTagStatsLimelight.setTagDetected(false);
  }

  @Override
  public boolean isFinished() {
    // drivetrain.stopModules();

    // if (!aprilTagStatsLimelight.hasValidTargets()
    //     ||(rotationPID.atSetpoint() && forwardPID.atSetpoint())) {
    //   drivetrain.stopModules();
    //   return true;
    // }
    //return false;
    return !aprilTagStatsLimelight.hasValidTargets();
  }
}
