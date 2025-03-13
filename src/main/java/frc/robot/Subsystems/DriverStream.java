// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriverStream extends SubsystemBase {
  /** Creates a new DriverStream. */
  Camera[] cameras;

  private ShuffleboardTab camTab = Shuffleboard.getTab("Camera");
  private VideoSink server;
  private int curCameraIndex = 0;

  public DriverStream(Camera... cameras) {
    // this.cameras = cameras;
    // server = CameraServer.getServer();
    // server.setSource(cameras[curCameraIndex].getCamera());

    // camTab.add("Camera Stream", server);
  }

  public void nextStream() {
    curCameraIndex++;
    if (curCameraIndex >= cameras.length) curCameraIndex = 0;
    server.setSource(cameras[curCameraIndex].getCamera());
  }

  public void previousStream() {
    curCameraIndex--;
    if (curCameraIndex < 0) curCameraIndex = cameras.length - 1;
    server.setSource(cameras[curCameraIndex].getCamera());
  }

  public int getCurrentCameraIndex() {
    return curCameraIndex;
  }
}
