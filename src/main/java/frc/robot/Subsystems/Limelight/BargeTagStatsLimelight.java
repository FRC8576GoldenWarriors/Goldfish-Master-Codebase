package frc.robot.Subsystems.Limelight;

import java.util.List;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Subsystems.Drivetrain;

public class BargeTagStatsLimelight extends SubsystemBase{
    private final NetworkTable table;
    private final Drivetrain drivetrain;
    

    public BargeTagStatsLimelight() {
        this.drivetrain = Drivetrain.getInstance();
        this.table = NetworkTableInstance.getDefault()
            .getTable(
                Constants.VisionConstants.limelightNetworkTableKey.BARGE_NETWORKTABLE_KEY
                );
    }

    public void updateStats() {
        double x = getTX();
        double y = getTY();
        double area = getArea();
        double id = getID();

        if(hasValidTargets()) {
            SmartDashboard.putBoolean("Barge Has Targets", true);
        } else {
            SmartDashboard.putBoolean("Barge Has Targets", false);
        }
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

    private void updateValues(double x, double y, double area, double id) {
        SmartDashboard.putNumber("Barge Limelight X", x);
        SmartDashboard.putNumber("Barge Limelight Y", y);
        SmartDashboard.putNumber("Barge Limelight Area", area);
        SmartDashboard.putNumber("Barge Limelight ID", id);
    }

    private void updateRobotPoseInSmartDashboard() {
        boolean hasTarget = hasValidTargets();
        SmartDashboard.putBoolean(" barge Limelight/Has Target", hasTarget);
    
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
        SmartDashboard.putNumber("Barge Limelight/Position/X", pose.getX());
        SmartDashboard.putNumber("Barge Limelight/Position/Y", pose.getY());
        SmartDashboard.putNumber("Barge Limelight/Position/Z", pose.getZ());
        SmartDashboard.putNumber("Barge Limelight/Rotation/Roll", Math.toDegrees(pose.getRotation().getX()));
        SmartDashboard.putNumber("Barge Limelight/Rotation/Pitch", Math.toDegrees(pose.getRotation().getY()));
        SmartDashboard.putNumber("Barge Limelight/Rotation/Yaw", Math.toDegrees(pose.getRotation().getZ()));
      }
    
      private void clearPoseDashboard() {
        SmartDashboard.putNumber("Barge Limelight/Position/X", 0);
        SmartDashboard.putNumber("Barge Limelight/Position/Y", 0);
        SmartDashboard.putNumber("Barge Limelight/Position/Z", 0);
        SmartDashboard.putNumber("Barge Limelight/Rotation/Roll", 0);
        SmartDashboard.putNumber("Barge Limelight/Rotation/Pitch", 0);
        SmartDashboard.putNumber("Barge Limelight/Rotation/Yaw", 0);
        SmartDashboard.putNumber("Barge Limelight/Distance", 0);
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
    
    public double calculateDistance() {
        int tagID = this.getID();
        if (tagID == -1) return 0.0;
        // List<Integer> bargeTagIDs = Constants.VisionConstants.aprilTagConstants.IDs.BARGE_TAG_IDS;
        // List<Integer> reefTagIDs = Constants.VisionConstants.aprilTagConstants.IDs.REEF_TAG_IDS;

        double cameraHeight = Constants.VisionConstants.limeLightDimensionConstants.CAMERA_HEIGHT;
        double cameraPitch = Constants.VisionConstants.limeLightDimensionConstants.CAMERA_PITCH;

        double heightGroundToTarget = getTagHeight(tagID);
        double angleTargetToDegrees = cameraPitch + getTY();
        double angleTargetToRadians = angleTargetToDegrees * (Math.PI / 180.0);
        double distanceFromTarget =
        (heightGroundToTarget - cameraHeight) / Math.tan(angleTargetToRadians);
        return distanceFromTarget;
  }

    public double getPitch() {
        Pose3d pose = getBotPose();
        return pose != null ? pose.getRotation().getY() : 0.0;
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
    
    private double getEntryValue(String entryName) {
        NetworkTableEntry val = table.getEntry(entryName);
        if (val == null) {
            return 0;
        }
        return val.getDouble(0.0);
   }
    
}
