// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Simulation;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SimEndEffector extends SubsystemBase {

  Mechanism2d arm;
  MechanismLigament2d base;
  MechanismRoot2d root;

  /** Creates a new SimEndEffector. */
  public SimEndEffector() {
    arm = new Mechanism2d(1, 0);
    root = arm.getRoot("arm", 0, 0);

    base = new MechanismLigament2d("Base", 5, 90);
    root.append(base);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putData("Arm", arm);
  }

  public void increaseAngle() {
    base.setAngle(base.getAngle() + 5);
  }

  public void decreaseAngle() {
    base.setAngle(base.getAngle() - 5);
  }
}
