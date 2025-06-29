package frc.robot.Subsystems.Limelight;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Subsystems.Drivetrain;
import org.littletonrobotics.junction.Logger;

public class AprilTagStatsLimelight extends SubsystemBase {

  private final Drivetrain drivetrain;
  private final NetworkTable table;
  private final String networkTableKey;
  private final String limelightName;

  private boolean tagIsDetected;
  private boolean tagIsReached;

  public AprilTagStatsLimelight(String networkTableKey) {
    this.drivetrain = Drivetrain.getInstance();
    this.networkTableKey = networkTableKey;
    this.limelightName = networkTableKey.substring(networkTableKey.indexOf("-") + 1);
    this.table =
        NetworkTableInstance.getDefault()
            .getTable(networkTableKey); // "limelight-reef"; // "limelight-processor";
    // configureAliance();

    tagIsReached = false;
    tagIsDetected = false;
  }

  public void updateStats() {
    double x = getTX();
    double y = getTY();
    double area = getArea();
    double id = getID();

    if (hasValidTargets()) {
      SmartDashboard.putBoolean(limelightName + " Has Targets", true);
      Logger.recordOutput(limelightName + "/Has Targets", true);
      updateRobotPoseInSmartDashboard();
    } else {
      SmartDashboard.putBoolean(limelightName + " Has Targets", false);
      Logger.recordOutput(limelightName + "/Has Targets", false);
    }

    updateValues(x, y, area, id);
  }

  public double getTX() {
    return getEntryValue("tx");
  }

  public double getTY() {
    return getEntryValue("ty");
  }

  public boolean hasValidTargets() {
    return getEntryValue("tv") == 1.0;
  }

  public double getArea() {
    return getEntryValue("ta");
  }

  public int getID() {
    if (hasValidTargets()) return (int) getEntryValue("tid");
    return -1;
  }

  public Pose3d getBotPose() {
    double[] botpose = table.getEntry("botpose").getDoubleArray(new double[6]);
    if (botpose.length < 6) {
      return null;
    }
    return new Pose3d(
        botpose[0],
        botpose[1],
        botpose[2],
        new Rotation3d(
            Math.toRadians(botpose[3]), Math.toRadians(botpose[4]), Math.toRadians(botpose[5])));
  }

  private double getEntryValue(String entryName) {
    NetworkTableEntry val = table.getEntry(entryName);
    if (val == null) {
      return 0;
    }
    return val.getDouble(0.0);
  }

  private void updateValues(double x, double y, double area, double id) {
    SmartDashboard.putNumber(limelightName + " Limelight X", x);
    SmartDashboard.putNumber(limelightName + " Limelight Y", y);
    SmartDashboard.putNumber(limelightName + " Limelight Area", area);
    SmartDashboard.putNumber(limelightName + " Limelight ID", id);

    Logger.recordOutput(limelightName + "/Limelight X", x);
    Logger.recordOutput(limelightName + "/Limelight Y", y);
    Logger.recordOutput(limelightName + "/Limelight Area", area);
    Logger.recordOutput(limelightName + "/Limelight ID", id);
  }

  public double getPitch() {
    Pose3d pose = getBotPose();
    return pose != null ? pose.getRotation().getY() : 0.0;
  }

  public double getTagHeight(int id) {
    if (Constants.VisionConstants.aprilTagConstants.IDs.REEF_TAG_IDS.contains(id))
      return Constants.VisionConstants.aprilTagConstants.heights.REEF_TAG_HEIGHT;
    else if (Constants.VisionConstants.aprilTagConstants.IDs.BARGE_TAG_IDS.contains(id))
      return Constants.VisionConstants.aprilTagConstants.heights.BARGE_TAG_HEIGHT;
    else if (Constants.VisionConstants.aprilTagConstants.IDs.PROCESSOR_TAG_IDS.contains(id))
      return Constants.VisionConstants.aprilTagConstants.heights.PROCESSOR_TAG_HEIGHT;
    else return Constants.VisionConstants.aprilTagConstants.heights.CORAL_STATION_TAG_HEIGHT;
  }

  public boolean isBargeLimelight() {
    // return networkTableKey.equals("limelight-barge");
    return true;
  }

  public boolean isBlueAlliance() {
    DriverStation.Alliance blueAlliane = DriverStation.Alliance.Blue;
    var currentAlliance = DriverStation.getAlliance();

    if (currentAlliance.isEmpty()) return false;
    else return blueAlliane.equals(currentAlliance.get());
  }

  public boolean isTagDetected() {
    return tagIsDetected;
  }

  public boolean isTagReached() {
    return tagIsReached;
  }

  public void setTagDetected(boolean isDetected) {
    tagIsDetected = isDetected;
  }

  public void setTagReached(boolean isReached) {
    tagIsReached = isReached;
  }

  private void updateRobotPoseInSmartDashboard() {
    boolean hasTarget = hasValidTargets();
    SmartDashboard.putBoolean(limelightName + " Limelight/Has Target", hasTarget);
    Logger.recordOutput(limelightName + "/Limelight Has Target", hasTarget);

    if (hasTarget) {
      Pose3d pose = getBotPose();
      if (pose != null) {
        updatePoseDashboard(pose);
      }
    } else {
      clearPoseDashboard();
    }
  }

  public boolean isConnected() {
    boolean connected = table != null;
    SmartDashboard.putBoolean("Limelight Connected", connected);
    Logger.recordOutput("Limelight/Connected", connected);
    return connected;
  }

  public double getDistance() {
    double focalLength = Constants.VisionConstants.LimelightConstants.FOCAL_LENGTH;
    double pixelWidth = Constants.VisionConstants.LimelightConstants.PIXEL_WIDTH;
    double callibrationFactor = 1.2;
    double realWidth = Constants.VisionConstants.LimelightConstants.REAL_WIDTH;
    double detectedWidth =
        realWidth * Math.sqrt(getArea()) / (pixelWidth * focalLength * callibrationFactor);

    double currentDistance =
        Math.abs(
            calculateDistance(
                    focalLength,
                    realWidth,
                    detectedWidth) // This is the distance from the camera to the target
                * 0.0002);

    currentDistance *=
        Math.cos(Math.toRadians(getTY())); // This is the distance from the bot to what the target's
    // wall (adjacent side)
    return currentDistance;
  }

  private void updatePoseDashboard(Pose3d pose) {
    SmartDashboard.putNumber("Limelight/Position/X", pose.getX());
    SmartDashboard.putNumber("Limelight/Position/Y", pose.getY());
    SmartDashboard.putNumber("Limelight/Position/Z", pose.getZ());
    SmartDashboard.putNumber("Limelight/Rotation/Roll", Math.toDegrees(pose.getRotation().getX()));
    SmartDashboard.putNumber("Limelight/Rotation/Pitch", Math.toDegrees(pose.getRotation().getY()));
    SmartDashboard.putNumber("Limelight/Rotation/Yaw", Math.toDegrees(pose.getRotation().getZ()));
    SmartDashboard.putNumber("Limelight/Distance", getDistance());

    Logger.recordOutput("Limelight/Position/X", pose.getX());
    Logger.recordOutput("Limelight/Position/Y", pose.getY());
    Logger.recordOutput("Limelight/Position/Z", pose.getZ());
    Logger.recordOutput("Limelight/Rotation/Roll", Math.toDegrees(pose.getRotation().getX()));
    Logger.recordOutput("Limelight/Rotation/Pitch", Math.toDegrees(pose.getRotation().getY()));
    Logger.recordOutput("Limelight/Rotation/Yaw", Math.toDegrees(pose.getRotation().getZ()));
    Logger.recordOutput("Limelight/Distance", getDistance());
  }

  private void clearPoseDashboard() {
    SmartDashboard.putNumber("Limelight/Position/X", 0);
    SmartDashboard.putNumber("Limelight/Position/Y", 0);
    SmartDashboard.putNumber("Limelight/Position/Z", 0);
    SmartDashboard.putNumber("Limelight/Rotation/Roll", 0);
    SmartDashboard.putNumber("Limelight/Rotation/Pitch", 0);
    SmartDashboard.putNumber("Limelight/Rotation/Yaw", 0);
    SmartDashboard.putNumber("Limelight/Distance", 0);

    Logger.recordOutput("Limelight/Position/X", 0);
    Logger.recordOutput("Limelight/Position/Y", 0);
    Logger.recordOutput("Limelight/Position/Z", 0);
    Logger.recordOutput("Limelight/Rotation/Roll", 0);
    Logger.recordOutput("Limelight/Rotation/Pitch", 0);
    Logger.recordOutput("Limelight/Rotation/Yaw", 0);
    Logger.recordOutput("Limelight/Distance", 0);
  }

  // ! Using the april tag area
  public double calculateDistance(double focalLength, double realWidth, double pixelWidth) {
    if (hasValidTargets()) {
      return (focalLength * realWidth) / pixelWidth;
    } else {
      return 0.0;
    }
  }

  // ! Using trig
  // public double calculateDistance() {
  //   int tagID = this.getID();
  //   SmartDashboard.putNumber("Tag id/calcDis", tagID);
  //   if (tagID == -1) return 0.0;
  //   // List<Integer> bargeTagIDs = Constants.VisionConstants.aprilTagConstants.IDs.BARGE_TAG_IDS;
  //   // List<Integer> reefTagIDs = Constants.VisionConstants.aprilTagConstants.IDs.REEF_TAG_IDS;

  //   double cameraHeight = Constants.VisionConstants.limeLightDimensionConstants.CAMERA_HEIGHT;
  //   double cameraPitch = Constants.VisionConstants.limeLightDimensionConstants.CAMERA_PITCH;

  //   double heightGroundToTarget = getTagHeight(tagID);
  //   double angleTargetToDegrees = cameraPitch + getTY();
  //   double angleTargetToRadians = angleTargetToDegrees * (Math.PI / 180.0);
  //   double distanceFromTarget =
  //   (cameraHeight - heightGroundToTarget) / Math.tan(angleTargetToRadians);
  //   return distanceFromTarget;
  // }

  // public void configureAliance(){
  //     var allianceColor = DriverStation.getAlliance();
  //     int targetTagID = (allianceColor.get() == Alliance.Blue) ?
  // Constants.VisionConstants.aprilTagIDConstants.BLUE_SPEAKER_TAG_ID :
  // Constants.VisionConstants.aprilTagIDConstants.RED_SPEAKER_TAG_ID;
  //     table.getEntry("pipeline").setNumber(targetTagID);
  // }

  public void updateLimelightTracking() {
    table.getEntry("camMode").setNumber(0); // Sets the vision processing mode
    table.getEntry("ledMode").setNumber(3); // Forces the LED to stay on always
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Limelight/Distance", getDistance());
    Logger.recordOutput("Limelight/Distance", getDistance());

    SmartDashboard.putBoolean("Tag is Reached", tagIsReached);
    SmartDashboard.putBoolean("Tag is Detected", tagIsDetected);
  }
}
