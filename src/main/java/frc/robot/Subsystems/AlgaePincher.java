// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.lib.drivers.WarriorSparkMax;
import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaePincher extends SubsystemBase {
  
  private WarriorSparkMax pincherMotor;
  private DigitalInput intakeSensor;

  public AlgaePincher() {

    pincherMotor = new WarriorSparkMax(
      Constants.AlgaeArmConstants.PincherConstants.pincherID, 
      MotorType.kBrushless, 
      Constants.AlgaeArmConstants.PincherConstants.motorIsInverted, 
      IdleMode.kBrake
    );

    intakeSensor = new DigitalInput(Constants.AlgaeArmConstants.PincherConstants.pincherDigiSensorID);
  }

  public void setPincherSpeed(double speed) {
    pincherMotor.set(speed);
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
