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

public class EndEffector extends SubsystemBase {

  private WarriorSparkMax pincherMotor;
  private DigitalInput coralDigitalInput;
  private DigitalInput algaeDigitalInput;

  public EndEffector() {

    pincherMotor =
        new WarriorSparkMax(
            Constants.EndEffectorConstants.HardwareConstants.pincherID,
            MotorType.kBrushless,
            Constants.EndEffectorConstants.HardwareConstants.motorIsInverted,
            IdleMode.kBrake,
            60);
    // coralDigitalInput =
    //     new DigitalInput(Constants.EndEffectorConstants.HardwareConstants.coralDigiSensorID);
    algaeDigitalInput =
        new DigitalInput(Constants.EndEffectorConstants.HardwareConstants.algaeDigiSensorID);
  }

  public void setSpeed(double speed) {
    pincherMotor.set(speed);
  }

  // public DigitalInput getCoralDigitalInput() {
  //   return coralDigitalInput;
  // }

  public DigitalInput getAlgaeDigitalInput() {
    return algaeDigitalInput;
  }

  public boolean motorRunning() {
    return pincherMotor.get() != 0;
  }

  public boolean getAlgaeDetected() {
    return !getAlgaeDigitalInput().get();
  }

  @Override
  public void periodic() {
    // Logger.recordOutput(
    //     "EndEffector/EndEffector_Coral_Digital_Input", getCoralDigitalInput().get());
    // Logger.recordOutput(
    //     "EndEffector/EndEffector_Algae_Digital_Input", getAlgaeDigitalInput().get());
    // Logger.recordOutput("EndEffector/Pinch_Motor_Voltage", pincherMotor.getBusVoltage());
    // Logger.recordOutput("EndEffector/Pinch_Motor_Current", pincherMotor.getOutputCurrent());
    // Logger.recordOutput("EndEffector/Pinch_Motor_Running", motorRunning());
  }
}
