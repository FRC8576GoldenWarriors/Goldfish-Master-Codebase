package frc.robot.Subsystems.Limelight;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Subsystems.Drivetrain;

public class AprilTagStatsLimelight extends SubsystemBase {

  private final Drivetrain drivetrain;
  private final NetworkTable table;

  // private AprilTagFieldLayout m_layout =
  // AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

  public AprilTagStatsLimelight() {
    this.drivetrain = Drivetrain.getInstance();
    this.table =
        NetworkTableInstance.getDefault()
            .getTable(
                Constants.VisionConstants.limelightNetworkTableKey.LIMELIGHT_NETWORKTABLE_KEY);
    // configureAliance();
  }

  public void updateStats() {
    double x = getTX();
    double y = getTY();
    double area = getArea();
    double id = getID();

    if (hasValidTargets()) {
      SmartDashboard.putBoolean("Has Targets", true);
      updateRobotPoseInSmartDashboard();
    } else {
      SmartDashboard.putBoolean("Has Targets", false);
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
    return getEntryValue("tv") == 1;
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
    SmartDashboard.putNumber("Limelight X", x);
    SmartDashboard.putNumber("Limelight Y", y);
    SmartDashboard.putNumber("Limelight Area", area);
    SmartDashboard.putNumber("Limelight ID", id);
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

  private void updateRobotPoseInSmartDashboard() {
    boolean hasTarget = hasValidTargets();
    SmartDashboard.putBoolean("Limelight/Has Target", hasTarget);

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

  // public double calculateDistance(int apriltagID){
  //     //Not meant for targets that are close to the same height as the camera
  //     if (apriltagID == -1) return 0;

  //     double TARGET_HEIGHT = this.getTagHeight(getID());
  //     double CAMERA_HEIGHT = Constants.VisionConstants.limeLightDimensionConstants.CAMERA_HEIGHT;
  //     double CAMERA_PITCH = Constants.VisionConstants.limeLightDimensionConstants.CAMERA_PITCH;

  //     double angleToSpeakerEntranceDegrees = Math.toRadians(CAMERA_PITCH + getTY());
  //     double heightDifferenceInches = (TARGET_HEIGHT - CAMERA_HEIGHT) * 39.37;

  //     return Math.abs((heightDifferenceInches) / Math.tan(angleToSpeakerEntranceDegrees))/39.97;
  // }

  public double calculateDistance(double focalLength, double realWidth, double pixelWidth) {
    if (hasValidTargets()) {
      return (focalLength * realWidth) / pixelWidth;
    } else {
      return 0.0;
    }
  }

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
