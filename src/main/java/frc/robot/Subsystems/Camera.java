package frc.robot.Subsystems;

import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.VideoSource;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;

public class Camera extends SubsystemBase {

  private HttpCamera camera;
  private PhotonCamera photonCamera;

  private static int currentPort = 1182;
  private int cameraPort;
  private String cameraName;
  private VideoSource.ConnectionStrategy connectionStrategy;

  public Camera(String cameraName, int resWidth, int resHeight, int fps, boolean keepAlive) {

    // this.cameraName = cameraName;
    // this.cameraPort = currentPort;

    // photonCamera = new PhotonCamera(cameraName);
    // if(photonCamera.isConnected()){
    // photonCamera.setDriverMode(true);

    // camera = new HttpCamera(cameraName, "http://10.85.76.11:" + cameraPort + "/stream.mjpg");

    // connectionStrategy =
    //     keepAlive
    //         ? VideoSource.ConnectionStrategy.kKeepOpen
    //         : VideoSource.ConnectionStrategy.kAutoManage;

    // camera.setConnectionStrategy(connectionStrategy);
    // camera.setResolution(resWidth, resHeight);
    // camera.setFPS(fps);

    // currentPort += 2;
    // }
  }

  public boolean isConnected() {
    return camera.isConnected() && photonCamera.isConnected();
  }

  public void setDriverMode(boolean cameraMode) {
    photonCamera.setDriverMode(cameraMode);
  }

  public HttpCamera getCamera() {
    return camera;
  }

  public int getCameraPort() {
    return cameraPort;
  }

  public void setCameraResolution(int resWidth, int resHeight) {
    camera.setResolution(resWidth, resHeight);
  }
}
