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

public class AprilTagStatsLimelight extends SubsystemBase {

  private final Drivetrain drivetrain;
  private final NetworkTable table;
  private final String networkTableKey;
  private final String limelightName;

  public AprilTagStatsLimelight(String networkTableKey) {
    this.drivetrain = Drivetrain.getInstance();
    this.networkTableKey = networkTableKey;
    this.limelightName = networkTableKey.substring(networkTableKey.indexOf("-") + 1);
    this.table =
        NetworkTableInstance.getDefault()
            .getTable(networkTableKey); // "limelight-reef"; // "limelight-processor";
    // configureAliance();
  }

  public void updateStats() {
    double x = getTX();
    double y = getTY();
    double area = getArea();
    double id = getID();

    if (hasValidTargets()) {
      SmartDashboard.putBoolean(limelightName + " Has Targets", true);
      updateRobotPoseInSmartDashboard();
    } else {
      SmartDashboard.putBoolean(limelightName + " Has Targets", false);
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
    return networkTableKey.equals("limelight-barge");
  }

  public boolean isBlueAlliance() {
    DriverStation.Alliance blueAlliane = DriverStation.Alliance.Blue;
    var currentAlliance = DriverStation.getAlliance();

    if (currentAlliance.isEmpty()) return false;
    else return blueAlliane.equals(currentAlliance.get());
  }

  private void updateRobotPoseInSmartDashboard() {
    boolean hasTarget = hasValidTargets();
    SmartDashboard.putBoolean(limelightName + " Limelight/Has Target", hasTarget);

    if (hasTarget) {
      Pose3d pose = getBotPose();
      if (pose != null) {
        updatePoseDashboard(pose);
      }
    } else {
      clearPoseDashboard();
    }
  }

  private void updatePoseDashboard(Pose3d pose) {
    SmartDashboard.putNumber("Limelight/Position/X", pose.getX());
    SmartDashboard.putNumber("Limelight/Position/Y", pose.getY());
    SmartDashboard.putNumber("Limelight/Position/Z", pose.getZ());
    SmartDashboard.putNumber("Limelight/Rotation/Roll", Math.toDegrees(pose.getRotation().getX()));
    SmartDashboard.putNumber("Limelight/Rotation/Pitch", Math.toDegrees(pose.getRotation().getY()));
    SmartDashboard.putNumber("Limelight/Rotation/Yaw", Math.toDegrees(pose.getRotation().getZ()));
  }

  private void clearPoseDashboard() {
    SmartDashboard.putNumber("Limelight/Position/X", 0);
    SmartDashboard.putNumber("Limelight/Position/Y", 0);
    SmartDashboard.putNumber("Limelight/Position/Z", 0);
    SmartDashboard.putNumber("Limelight/Rotation/Roll", 0);
    SmartDashboard.putNumber("Limelight/Rotation/Pitch", 0);
    SmartDashboard.putNumber("Limelight/Rotation/Yaw", 0);
    SmartDashboard.putNumber("Limelight/Distance", 0);
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
}
