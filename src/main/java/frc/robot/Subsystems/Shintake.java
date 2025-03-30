// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drivers.WarriorSparkMax;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class Shintake extends SubsystemBase {

  private WarriorSparkMax lowerRollerMotor;
  private WarriorSparkMax upperRollerMotor;

  private WarriorSparkMax pivotMotor;

  private DigitalInput lowerRollerDigitalInput;

  private boolean isRevved;

  public Shintake() {
    lowerRollerMotor =
        new WarriorSparkMax(
            Constants.ShintakeConstants.HardwareConstants.rollerMotorLowID,
            MotorType.kBrushless,
            Constants.ShintakeConstants.HardwareConstants.rollerMotorLowIsInverted,
            IdleMode.kCoast,
            60);

    upperRollerMotor =
        new WarriorSparkMax(
            Constants.ShintakeConstants.HardwareConstants.rollerMotorHighID,
            MotorType.kBrushless,
            Constants.ShintakeConstants.HardwareConstants.rollerMotorHighIsInverted,
            IdleMode.kCoast,
            60);

    lowerRollerDigitalInput =
        new DigitalInput(Constants.ShintakeConstants.HardwareConstants.lowerRollerDigitalInputDIO);

    // DO NOT CHANGE WITHOUT OFFICER/DEPUTY SUPERVISION
    lowerRollerMotor.setkF(1 / 5800.0); // 1/4730.0
    lowerRollerMotor.setkP(0.0003); // 0.0005
    lowerRollerMotor.setkI(0.0);
    lowerRollerMotor.setkD(0.000003);
    lowerRollerMotor.setMaxMotion(5600, 12000);

    upperRollerMotor.setkF(1 / 6100.0);
    upperRollerMotor.setkP(0.0003);
    upperRollerMotor.setkI(0.0);
    upperRollerMotor.setkD(0.000003);
    upperRollerMotor.setMaxMotion(5600, 12000);
  }

  @Override
  public void periodic() {

    Logger.recordOutput("Shintake/Lower_Roller_Motor_Voltage", lowerRollerMotor.getBusVoltage());
    Logger.recordOutput("Shintake/Lower_Roller_Motor_Current", lowerRollerMotor.getOutputCurrent());
    Logger.recordOutput(
        "Shintake/Lower_Roller_Encoder_RPM", lowerRollerMotor.getEncoder().getVelocity());
    Logger.recordOutput("Shintake/Upper_Roller_Motor_Voltage", upperRollerMotor.getBusVoltage());
    Logger.recordOutput("Shintake/Upper_Roller_Motor_Current", upperRollerMotor.getOutputCurrent());
    Logger.recordOutput(
        "Shintake/Upper_Roller_Encoder_RPM", upperRollerMotor.getEncoder().getVelocity());

    SmartDashboard.putNumber(
        "Shintake Lower Roller RPM", lowerRollerMotor.getEncoder().getVelocity());
    SmartDashboard.putNumber(
        "Shintake Upper Roller RPM", upperRollerMotor.getEncoder().getVelocity());

    SmartDashboard.putBoolean("Lower Roller Digital Input", getAlgaeDetected());
  }

  public void setPivotSpeed(double speed) {
    pivotMotor.set(speed);
  }

  public void setPivotVoltage(double voltage) {
    pivotMotor.setVoltage(voltage);
  }

  public void setLowerRollerSpeed(double speed) {
    lowerRollerMotor.set(speed);
  }

  public void setLowerRollerVoltage(double voltage) {
    lowerRollerMotor.setVoltage(voltage);
  }

  public void setUpperRollerSpeed(double speed) {
    upperRollerMotor.set(speed);
  }

  public void setUpperRollerVoltage(double voltage) {
    upperRollerMotor.setVoltage(voltage);
  }

  public void setRollersSpeed(double speed) {
    setLowerRollerSpeed(speed); // -0.95 *
    setUpperRollerSpeed(speed);
  }

  public void setRollersSpeed(double lowRollerSpeed, double upperRollerSpeed) {
    setLowerRollerSpeed(lowRollerSpeed);
    setUpperRollerSpeed(upperRollerSpeed);
  }

  public void setRollersVoltage(double voltage) {
    setLowerRollerVoltage(voltage);
    setUpperRollerVoltage(voltage);
  }

  public void setRollerRPMs(double lowerRPM, double upperRPM) {
    lowerRollerMotor
        .getClosedLoopController()
        .setReference(lowerRPM, ControlType.kMAXMotionVelocityControl);
    upperRollerMotor
        .getClosedLoopController()
        .setReference(upperRPM, ControlType.kMAXMotionVelocityControl);
  }

  public double getAverageEncoderVelocity() {
    return (lowerRollerMotor.getEncoder().getVelocity()
            + upperRollerMotor.getEncoder().getVelocity())
        / 2.0;
  }

  public WarriorSparkMax getUpperRollerMotor() {
    return upperRollerMotor;
  }

  public WarriorSparkMax getLowerRollerMotor() {
    return lowerRollerMotor;
  }

  public boolean getAlgaeDetected() {
    return !lowerRollerDigitalInput.get();
  }

  public void setIsRevved(boolean isRevved) {
    this.isRevved = isRevved;
  }

  public boolean getIsRevved() {
    return isRevved;
  }
}
