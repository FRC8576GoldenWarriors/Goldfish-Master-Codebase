// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drivers.WarriorSparkMax;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class Shintake extends SubsystemBase {

  private WarriorSparkMax lowerRollerMotor;
  private WarriorSparkMax upperRollerMotor;

  private WarriorSparkMax pivotMotor;

  private DutyCycleEncoder encoder;

  private InterpolatingDoubleTreeMap RPMtoVoltage;

  private boolean isRevved;

  public Shintake() {
    lowerRollerMotor =
        new WarriorSparkMax(
            Constants.ShintakeConstants.HardwareConstants.rollerMotorLowID,
            MotorType.kBrushless,
            Constants.ShintakeConstants.HardwareConstants.rollerMotorLowIsInverted,
            IdleMode.kCoast,
            70);

    upperRollerMotor =
        new WarriorSparkMax(
            Constants.ShintakeConstants.HardwareConstants.rollerMotorHighID,
            MotorType.kBrushless,
            Constants.ShintakeConstants.HardwareConstants.rollerMotorHighIsInverted,
            IdleMode.kCoast,
            70);

    RPMtoVoltage = new InterpolatingDoubleTreeMap(); // use to interpolate (volts, rpm) values

    RPMtoVoltage.put(Double.valueOf(0), Double.valueOf(0)); // (0 rpm, 0 voltage)
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Logger.recordOutput("Shintake/IsRevved", getIsRevved());
    Logger.recordOutput("Shintake/Average_Encoder_Velocity", getAverageEncoderVelocity());
    Logger.recordOutput("Shintake/Lower_Roller_Motor_Voltage", lowerRollerMotor.getBusVoltage());
    Logger.recordOutput("Shintake/Lower_Roller_Motor_Current", lowerRollerMotor.getOutputCurrent());
    Logger.recordOutput(
        "Shintake/Lower_Roller_Encoder_Position", lowerRollerMotor.getEncoder().getVelocity());
    Logger.recordOutput("Shintake/Upper_Roller_Motor_Voltage", upperRollerMotor.getBusVoltage());
    Logger.recordOutput("Shintake/Upper_Roller_Motor_Current", upperRollerMotor.getOutputCurrent());
    Logger.recordOutput(
        "Shintake/Upper_Roller_Encoder_Position", upperRollerMotor.getEncoder().getVelocity());
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
    setLowerRollerSpeed(-lowRollerSpeed);
    setUpperRollerSpeed(-upperRollerSpeed);
  }

  public void setRollersVoltage(double voltage) {
    setLowerRollerVoltage(voltage);
    setUpperRollerVoltage(voltage);
  }

  public double getAverageEncoderVelocity() {
    return (lowerRollerMotor.getEncoder().getVelocity()
            + upperRollerMotor.getEncoder().getVelocity())
        / 2.0;
  }

  public double calculateMotorVoltage(double rpm) {
    return RPMtoVoltage.get(Double.valueOf(rpm));
  }

  public DutyCycleEncoder getEncoder() {
    return encoder;
  }

  public void setIsRevved(boolean isRevved) {
    this.isRevved = isRevved;
  }

  public boolean getIsRevved() {
    return isRevved;
  }
}
