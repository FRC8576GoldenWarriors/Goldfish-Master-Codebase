// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drivers.WarriorSparkMax;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private WarriorSparkMax intakeMotor;
  private DutyCycleEncoder armAbsoluteEncoder;
  private RelativeEncoder armRelativeEncoder;

  public Intake() {
    intakeMotor = new WarriorSparkMax(Constants.IntakeConstants.HardwareConstants.pivotMotorId,
     MotorType.kBrushless, false, IdleMode.kCoast,45);
    
     armRelativeEncoder = intakeMotor.getEncoder();

    armAbsoluteEncoder = new DutyCycleEncoder(Constants.IntakeConstants.HardwareConstants.pivotAbsoluteEncoder,2*Math.PI,
    Constants.IntakeConstants.HardwareConstants.pivotEncoderZero);

    armAbsoluteEncoder.setInverted(true);
    
  }

  public double getAbsoluteEncoderPosition(){
    return armAbsoluteEncoder.get();
    }
  public double getRelativeEncoder(){
    return armRelativeEncoder.getPosition();
  }
  public void setVoltage(double voltage){
    intakeMotor.setVoltage(voltage);
  }

  public void setMotorSpeed(double speed){
    intakeMotor.set(speed);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Relative Encoder",getRelativeEncoder());
    SmartDashboard.putNumber("Absolute Encoder", getAbsoluteEncoderPosition());
    SmartDashboard.putNumber("Voltage Output", intakeMotor.getAppliedOutput());
    // This method will be called once per scheduler run
  }
}

