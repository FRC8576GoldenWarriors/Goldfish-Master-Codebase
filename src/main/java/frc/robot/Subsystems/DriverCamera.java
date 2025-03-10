package frc.robot.Subsystems;

import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriverCamera extends SubsystemBase {
    
    PhotonCamera driverCamera;
    PhotonCamera cageCamera;

    public DriverCamera(){
        driverCamera = new PhotonCamera(Constants.VisionConstants.DriverCameraConstants.DRIVER_CAMERA_NT_KEY);
        cageCamera = new PhotonCamera(Constants.VisionConstants.DriverCameraConstants.CAGE_CAMERA_NT_KEY);
    }
}
