// package frc.robot.Subsystems;

// import java.util.List;

// import org.photonvision.PhotonCamera;
// import org.photonvision.PhotonPoseEstimator;
// import org.photonvision.PhotonPoseEstimator.PoseStrategy;
// import org.photonvision.PhotonUtils;
// import org.photonvision.targeting.PhotonTrackedTarget;

// import com.pathplanner.lib.path.GoalEndState;
// import com.pathplanner.lib.path.PathConstraints;
// import com.pathplanner.lib.path.PathPlannerPath;
// import com.pathplanner.lib.path.Waypoint;

// import edu.wpi.first.apriltag.AprilTagFieldLayout;
// import edu.wpi.first.apriltag.AprilTagFields;
// import edu.wpi.first.math.Matrix;
// import edu.wpi.first.math.Nat;
// import edu.wpi.first.math.VecBuilder;
// import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Pose3d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Rotation3d;
// import edu.wpi.first.math.geometry.Transform3d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.geometry.Translation3d;
// import edu.wpi.first.math.numbers.N3;
// import edu.wpi.first.networktables.GenericEntry;
// import edu.wpi.first.networktables.NetworkTableInstance;
// import edu.wpi.first.networktables.StructPublisher;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
// import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
// import edu.wpi.first.wpilibj.smartdashboard.Field2d;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants.*;
// import frc.robot.Subsystems.Drivetrain;
// public class AprilTagStats extends SubsystemBase {
//     //Creating new object for the arducam
//     private PhotonCamera m_arduCam;

//     //Object representation of the field
//     private AprilTagFieldLayout m_layout =
// AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

//     //Positional object representation of the camera on the robot
//     private final Transform3d m_robotToCam = new Transform3d(new
// Translation3d(Constants.VisionConstants.cameraTranslationConstants.tX,
//     Constants.VisionConstants.cameraTranslationConstants.tY,
//     Constants.VisionConstants.cameraTranslationConstants.tZ),
//     new Rotation3d(Constants.VisionConstants.cameraRotationConstants.rRoll,
//     Constants.VisionConstants.cameraRotationConstants.rPitch,
//     Constants.VisionConstants.cameraRotationConstants.rYaw));

//     //Photon Pose Estimator object
//     private final Drivetrain m_drivetrain = Drivetrain.getInstance();
//     //Shuffleboard tab named vision, and 4 different widgets for yaw, pitch, tag id, and distance
// to a tag
//     private ShuffleboardTab m_tab;
//     private GenericEntry m_yawEntry, m_pitchEntry, m_idEntry, m_distanceEntry, m_tagStatusEntry;
//     //Variables to hold all of the widget values
//     private double m_yaw, m_pitch, m_distance;
//     private int m_id;
//     private boolean m_tagStatus;

//     //Generic starting position of the robot
//     // private final Pose3d m_startPose3d = new Pose3d(12.5, 5.5, 0, new Rotation3d());
//     private StructPublisher<Pose2d> m_publisher;
//     private PhotonTrackedTarget target;
//     private static SwerveDrivePoseEstimator poseEstimator;
//     private Field2d field;
//     private Pose2d relativePose;

//     public AprilTagStats(String cameraName, String publishName, String tabName) {
//         m_publisher = NetworkTableInstance.getDefault().getStructTopic(publishName,
// Pose2d.struct).publish();
//         m_tab = Shuffleboard.getTab(tabName);
//         m_arduCam = new PhotonCamera(cameraName);
//         m_yawEntry = m_tab.add("yaw", m_yaw).getEntry();
//         m_pitchEntry = m_tab.add("pitch", m_pitch).getEntry();
//         m_idEntry = m_tab.add("id", m_id).getEntry();
//         m_distanceEntry = m_tab.add("distance", m_distance).getEntry();
//         m_tagStatusEntry = m_tab.add("Tag In View", m_tagStatus).getEntry();
//         field = new Field2d();
//         SmartDashboard.putData("Field2d",field);
//         poseEstimator =
//         new SwerveDrivePoseEstimator(
//             Constants.SwerveConstants.DRIVE_KINEMATICS,
//             m_drivetrain.getHeadingRotation2d(),
//             m_drivetrain.getModulePositions(),
//             m_drivetrain.getPose2d(),
//             new Matrix(VecBuilder.
//             fill(
//                 // if these numbers are less than one, multiplying will do bad things
//                 Constants.SwerveConstants.PoseConfig.kVisionStdDevX,
//                 Constants.SwerveConstants.PoseConfig.kVisionStdDevY,
//                 Constants.SwerveConstants.PoseConfig.kVisionStdDevTheta
//             )),

//             new Matrix(VecBuilder
//             .fill(
//                 // if these numbers are less than one, multiplying will do bad things
//                 Constants.SwerveConstants.PoseConfig.kPositionStdDevX,
//                 Constants.SwerveConstants.PoseConfig.kPositionStdDevY,
//                 Constants.SwerveConstants.PoseConfig.kPositionStdDevTheta
//             )));
//     }
//     @Override
//   public void periodic() {
//     poseEstimator.update(m_drivetrain.getHeadingRotation2d(),m_drivetrain.getModulePositions());
// // Updates using wheel encoder data only
//     // Updates using the vision estimate

//       double currentTimestamp =
//           getTimeStamp();
//       if (Constants.VisionConstants.distanceConstants.useableIDs.contains(getID())) {
//         poseEstimator.addVisionMeasurement(getRobotPose(), currentTimestamp);
//       }
//       field.setRobotPose(poseEstimator.getEstimatedPosition());
// }
//     private PhotonTrackedTarget getTarget() {
//         return m_arduCam.getLatestResult().getBestTarget();
//     }

//     public boolean pingCam() {
//         return m_arduCam.isConnected();
//     }

//     public void updateData() {
//         //pushes yaw, pitch, id, and distance between the robot and tag to ShuffleBoard (Meant
// for testing if values are being passed to variables)
//         m_yawEntry.setDouble(getYaw());
//         m_pitchEntry.setDouble(getPitch());
//         m_idEntry.setInteger(getID());
//         m_distanceEntry.setDouble(getDistance());
//     }

//     public Pose2d getRobotPose() {
//         //intakes the space between a camera and its target, the target itself, the camera's
// position and rotation on the robot, and the field. Outputs the robot relative to the field.
//         target = getTarget();
//         Pose3d estimatedPose = new Pose3d();
//         int id = getID();
//         if (id != -1 && target != null && m_layout.getTagPose(id).isPresent()) {
//             estimatedPose =
// PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(),
// m_layout.getTagPose(id).get(), m_robotToCam);
//         }
//         return estimatedPose.toPose2d();
//     }
//     public PathPlannerPath robotPath(double angle){
//         List<Waypoint> waypoints = null;
//         Transform3d tagTransform = getTransform3d();
//         double xTransform =
// tagTransform.getX()-Constants.VisionConstants.distanceConstants.goalMeterDistance+poseEstimator.getEstimatedPosition().getX();
//         double yTransform = tagTransform.getY()+poseEstimator.getEstimatedPosition().getY();
//         Rotation2d rotTransform = tagTransform.getRotation().toRotation2d();
//         if(Constants.VisionConstants.distanceConstants.useableIDs.contains(m_id)){
//         waypoints = PathPlannerPath.waypointsFromPoses(
//         getRobotPose(),
//         new Pose2d(xTransform,yTransform,rotTransform)
// );
//         }
// //         else if(m_id==4){
// //         waypoints = PathPlannerPath.waypointsFromPoses(
// //         getRobotPose(),
// //         new
// Pose2d(tagPose.getX()-Constants.VisionConstants.distanceConstants.goalMeterDistance,tagPose.getY(),tagPose.getRotation())
// // );
// //         }

// PathConstraints constraints = new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI); // The
// constraints for this path.
// // PathConstraints constraints = PathConstraints.unlimitedConstraints(12.0); // You can also use
// unlimited constraints, only limited by motor torque and nominal battery voltage

// // Create the path using the waypoints created above
// if(waypoints!=null){
// PathPlannerPath path = new PathPlannerPath(
//         waypoints,
//         constraints,
//         null,
//         new GoalEndState(0.0, Rotation2d.fromDegrees(angle))
// );
// path.preventFlipping = true;
// return path;
// }
// return null;
//     }
//     public void updatePose() {
//         //Updates the robots pose on network tables
//         m_publisher.set(getRobotPose());
//     }

//     public double getDistance() {
//         //Gets the distance between the robot and the wall of the apriltag its viewing
//         double tagPitch = getPitch();
//         relativePose = getRobotPose();
//         int id = getID();
//         if (id != -1 && relativePose != null) {
//             double hypotenuse = PhotonUtils.getDistanceToPose(relativePose,
// m_layout.getTagPose(getID()).get().toPose2d());
//             return hypotenuse * Math.cos(Math.toRadians(tagPitch));
//         }
//         return 0;
//     }

//     public double getYaw() {
//         target = getTarget();
//         if (target != null) {
//             return target.getYaw();
//         }
//         return 0;
//     }

//     public double getPitch() {
//         target = getTarget();
//         if (target != null) {
//             return target.getPitch();
//         }
//         return 0;
//     }

//     public int getID() {
//         target = getTarget();
//         if (target != null) {
//             return target.getFiducialId();
//         }
//         return -1;
//     }

//     public boolean hasTarget() {
//         if (m_arduCam.getLatestResult().hasTargets() && pingCam()) {
//             return true;
//         }
//         return false;
//     }

//     public void setTagView(boolean tagFound) {
//         if (tagFound) {
//             m_tagStatusEntry.setBoolean(true);
//         }
//         else {
//             m_tagStatusEntry.setBoolean(false);
//         }
//     }
//     public double getTimeStamp(){
//         return m_arduCam.getLatestResult().getTimestampSeconds();
//     }

//     public Transform3d getTransform3d(){
//         return getTarget().getBestCameraToTarget();
//     }

//     public void updateView() {
//         // TODO Auto-generated method stub
//         throw new UnsupportedOperationException("Unimplemented method 'updateView'");
//     }
// }
