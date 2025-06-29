// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drivers.WarriorSparkMax;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class GroundIntake extends SubsystemBase {

  private WarriorSparkMax pivotMotor;
  private WarriorSparkMax rollerMotor;

  private DutyCycleEncoder encoder;

  private DigitalInput algaeSensor;

  private double error;

  public GroundIntake() {
    pivotMotor =
        new WarriorSparkMax(
            Constants.GroundIntakeConstants.HardwareConstants.pivotMotorID,
            MotorType.kBrushless,
            Constants.GroundIntakeConstants.HardwareConstants.pivotMotorIsInverted,
            IdleMode.kBrake,
            30);

    rollerMotor =
        new WarriorSparkMax(
            Constants.GroundIntakeConstants.HardwareConstants.rollerMotorID,
            MotorType.kBrushless,
            Constants.GroundIntakeConstants.HardwareConstants.rollerMotorIsInverted,
            IdleMode.kBrake,
            22);

    encoder =
        new DutyCycleEncoder(
            Constants.GroundIntakeConstants.HardwareConstants.pivotEncoderDIO,
            Constants.GroundIntakeConstants.ControlConstants.pivotEncoderFullRange,
            Constants.GroundIntakeConstants.ControlConstants.pivotEncoderZero);

    encoder.setInverted(Constants.GroundIntakeConstants.ControlConstants.pivotEncoderIsInverted);

    algaeSensor =
        new DigitalInput(Constants.GroundIntakeConstants.HardwareConstants.digitalInputDIO);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Logger.recordOutput("Ground_Intake/Ground_Intake_Position", getEncoderPosition());
    Logger.recordOutput("Ground_Intake/Algae Detected", getAlgaeDetected());
    Logger.recordOutput(
        "Ground_Intake/Roller Motor Applied Voltage", rollerMotor.getAppliedOutput());
    Logger.recordOutput("Ground_Intake/Roller Motor Current", rollerMotor.getOutputCurrent());
    SmartDashboard.putNumber("GroundIntake/encoderPosition", encoder.get());
    SmartDashboard.putBoolean("GroundIntake/Hold Photoelectric", getAlgaeDetected());
    SmartDashboard.putBoolean("GroundIntake/Encoder Connected", encoder.isConnected());
  }

  public void setPivotSpeed(double speed) {
    pivotMotor.set(speed);
  }

  public void setPivotVoltage(double voltage) {
    pivotMotor.setVoltage(voltage);
  }

  public void setRollerSpeed(double speed) {
    rollerMotor.set(speed);
  }

  public void setError(double error) {
    this.error += error;
  }

  public double getError() {
    return error;
  }

  public void setRollerVoltage(double voltage) {
    rollerMotor.setVoltage(voltage);
  }

  public DutyCycleEncoder getEncoder() {
    return encoder;
  }

  public double getEncoderPosition() {
    return getEncoder().get();
  }

  public boolean getAlgaeDetected() {
    return !algaeSensor.get();
  }

  public double getPivotVelocity() {
    return pivotMotor.getEncoder().getVelocity();
  }
}
