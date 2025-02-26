// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drivers.WarriorSparkMax;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
  /** Creates a new AlgaeArm. */
  private WarriorSparkMax armMotor;

  private DutyCycleEncoder armAbsEncoder;

  public Arm() {
    armMotor =
        new WarriorSparkMax(
            Constants.ArmConstants.HardwareConstants.armMotorID,
            MotorType.kBrushless,
            Constants.ArmConstants.ControlConstants.motorIsInverted,
            IdleMode.kCoast,
            45);

    armAbsEncoder =
        new DutyCycleEncoder(
            Constants.ArmConstants.HardwareConstants.armEncoderDIO,
            1.0,
            Constants.ArmConstants.ControlConstants.armEncoderOffset);

    armAbsEncoder.setInverted(Constants.ArmConstants.ControlConstants.armEncoderIsInverted);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    Logger.recordOutput("Arm/Arm_Voltage", armMotor.getBusVoltage());
    Logger.recordOutput("Arm/Arm_Current", armMotor.getOutputCurrent());
    Logger.recordOutput("Arm/Arm_Encoder_Value", getEncoderPosition());
    Logger.recordOutput("Arm/Arm_Velocity", getArmVelocity());

    SmartDashboard.putNumber("Arm Enocder Position", getEncoderPosition());
    SmartDashboard.putNumber("Arm Motor Voltage", armMotor.getBusVoltage());
    SmartDashboard.putNumber("Arm Encoder Velocity", getArmVelocity());
  }

  public void setArmVoltage(double volts) {
    armMotor.setVoltage(volts);
  }

  public void setArmSpeed(double speed) {
    armMotor.set(speed);
  }

  public SparkMax getMotor() {
    return armMotor;
  }

  // radians per second
  public double getArmVelocity() {
    return (getMotor().getEncoder().getVelocity() / 60.0); // rpm/60
  }

  public DutyCycleEncoder getDutyCycleEncoder() {
    return armAbsEncoder;
  }

  public double getEncoderPosition() {
    return armAbsEncoder.get();
  }

  public DutyCycleEncoder getEncoder() {
    return armAbsEncoder;
  }

  public void setArmMotorIdleMode(IdleMode idleMode) {
    armMotor.setIdleMode(idleMode);
  }
}