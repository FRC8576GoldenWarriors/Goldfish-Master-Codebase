// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaePincher extends SubsystemBase {
  
  private SparkMax algaePincher;
  private DigitalInput intakeSensor;

  public AlgaePincher() {
    algaePincher = new SparkMax(Constants.AlgaeArmConstants.PincherConstants.pincherID, MotorType.kBrushless);
    intakeSensor = new DigitalInput(Constants.AlgaeArmConstants.PincherConstants.pincherDigiSensorID);
  }

  public void setPincherSpeed(double speed) {
    algaePincher.set(speed);
  }
  public DigitalInput getDigitalInput(){
    return intakeSensor;
  }
  public boolean hasAlgae() {
    return intakeSensor.get();
  }

  @Override
  public void periodic() {
  }
}
