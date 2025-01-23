// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drivers.WarriorSparkMax;
import frc.robot.Constants;

public class CoralRoller extends SubsystemBase {

  private WarriorSparkMax coralRoller;
  private DigitalInput intakeSensor;

  public CoralRoller() {
    coralRoller =
        new WarriorSparkMax(
            Constants.CoralRollerConstants.coralRollerID,
            MotorType.kBrushless,
            Constants.CoralRollerConstants.motorIsInverted,
            IdleMode.kBrake);

    intakeSensor = new DigitalInput(Constants.CoralRollerConstants.coralRollerDigiSensorID);
  }

  public void setRollerSpeed(double speed) {
    coralRoller.set(speed);
  }

  public DigitalInput getDigitalInput() {
    return intakeSensor;
  }

  public boolean hasCoral() {
    return intakeSensor.get();
  }

  @Override
  public void periodic() {}
}
