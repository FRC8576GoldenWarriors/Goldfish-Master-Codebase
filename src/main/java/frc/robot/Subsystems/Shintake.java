// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drivers.WarriorSparkMax;
import frc.robot.Constants;

public class Shintake extends SubsystemBase {
  
  private WarriorSparkMax lowerRollerMotor;
  private WarriorSparkMax upperRollerMotor;

  private WarriorSparkMax pivotMotor;

  private DutyCycleEncoder encoder;


  public Shintake() {
    lowerRollerMotor = new WarriorSparkMax(
      Constants.ShintakeConstants.HardwareConstants.rollerMotorLowID,
      MotorType.kBrushless, 
      Constants.ShintakeConstants.HardwareConstants.rollerMotorLowIsInverted,
      IdleMode.kBrake
    );

    upperRollerMotor = new WarriorSparkMax(
      Constants.ShintakeConstants.HardwareConstants.rollerMotorHighID,
      MotorType.kBrushless, 
      Constants.ShintakeConstants.HardwareConstants.rollerMotorHighIsInverted,
      IdleMode.kBrake
    );

    pivotMotor = new WarriorSparkMax(
      Constants.ShintakeConstants.HardwareConstants.pivotMotorID,
      MotorType.kBrushless, 
      Constants.ShintakeConstants.HardwareConstants.pivotMotorIsInverted,
      IdleMode.kBrake
    );

    encoder = new DutyCycleEncoder(
      Constants.ShintakeConstants.HardwareConstants.pivotMotorEncoderDIO,
      Constants.ShintakeConstants.HardwareConstants.pivotEncoderFullRange,
      Constants.ShintakeConstants.HardwareConstants.pivotEncoderZero
    );


  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setPivotSpeed(double speed){
    pivotMotor.set(speed);
  }

  public void setPivotVoltage(double voltage){
    pivotMotor.setVoltage(voltage);
  }



  public void setLowerRollerSpeed(double speed){
    lowerRollerMotor.set(speed);
  }
  
  public void setLowerRollerVoltage(double voltage){
    lowerRollerMotor.setVoltage(voltage);
  }
  

  public void setUpperRollerSpeed(double speed){
    upperRollerMotor.set(speed);
  }

  public void setUpperRollerVoltage(double voltage){
    upperRollerMotor.setVoltage(voltage);
  }

  public DutyCycleEncoder getEncoder(){
    return encoder;
  }

}
