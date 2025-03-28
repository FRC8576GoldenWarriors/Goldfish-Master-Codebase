// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drivers.WarriorSparkMax;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class EndEffector extends SubsystemBase {

  private WarriorSparkMax pincherMotor;
  private DigitalInput coralLeftDigitalInput;
  private DigitalInput algaeDigitalInput;

  public EndEffector() {

    pincherMotor =
        new WarriorSparkMax(
            Constants.EndEffectorConstants.HardwareConstants.pincherID,
            MotorType.kBrushless,
            Constants.EndEffectorConstants.HardwareConstants.motorIsInverted,
            IdleMode.kBrake,
            60);

    algaeDigitalInput =
        new DigitalInput(Constants.EndEffectorConstants.HardwareConstants.algaeDigiSensorID);

    coralLeftDigitalInput =
        new DigitalInput(Constants.EndEffectorConstants.HardwareConstants.coralLeftDigiSensorID);
  }

  public void setSpeed(double speed) {
    pincherMotor.set(speed);
  }

  public DigitalInput getLeftCoralDigitalInput() {
    return coralLeftDigitalInput;
  }

  public DigitalInput getAlgaeDigitalInput() {
    return algaeDigitalInput;
  }

  public boolean motorRunning() {
    return pincherMotor.get() != 0;
  }

  public boolean getAlgaeDetected() {
    return !getAlgaeDigitalInput().get();
  }

  public boolean getCoralDetected() {
    return getLeftCoralDetected();
  }

  public boolean getLeftCoralDetected() {
    return !getLeftCoralDigitalInput().get();
  }

  @Override
  public void periodic() {

    Logger.recordOutput(
        "EndEffector/EndEffector_Algae_Digital_Input", getAlgaeDigitalInput().get());

    Logger.recordOutput("EndEffector/Pinch_Motor_Voltage", pincherMotor.getBusVoltage());
    Logger.recordOutput("EndEffector/Pinch_Motor_Current", pincherMotor.getOutputCurrent());
    Logger.recordOutput("EndEffector/Pinch_Motor_Running", motorRunning());

    SmartDashboard.putBoolean("Left Coral Digital Input", getLeftCoralDetected());
  }
}
