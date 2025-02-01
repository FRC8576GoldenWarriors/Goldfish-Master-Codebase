package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.photonvision.PhotonCamera;

public class DriverCamera extends SubsystemBase {

  private PhotonCamera m_arduCam;
  private ShuffleboardTab m_tab;

  public DriverCamera(String cameraName) {
    m_tab = Shuffleboard.getTab(Constants.VisionConstants.nameConstants.tabName);
    m_arduCam = new PhotonCamera(cameraName);
    m_arduCam.setDriverMode(Constants.VisionConstants.driverMode);
    // m_tab.addCamera("Driver Cam", cameraName, "http://10.85.76.11:1182/?action=stream");
  }

  public boolean pingDriverCamera() {
    return m_arduCam.isConnected() && m_arduCam.getDriverMode();
  }

  public void setDriverMode(boolean driverMode) {
    m_arduCam.setDriverMode(driverMode);
  }
}
