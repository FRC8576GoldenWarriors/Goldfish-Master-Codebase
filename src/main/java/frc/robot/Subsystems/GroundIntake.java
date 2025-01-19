// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drivers.WarriorSparkMax;
import frc.robot.Constants;


public class GroundIntake extends SubsystemBase {

  private WarriorSparkMax pivotMotor;
  private WarriorSparkMax rollerMotor;

  private DutyCycleEncoder encoder;

  private DigitalInput algaeSensor;


  public GroundIntake() {
    pivotMotor = new WarriorSparkMax(
      Constants.GroundIntakeConstants.HardwareConstants.pivotMotorID,
      MotorType.kBrushless,
      Constants.GroundIntakeConstants.HardwareConstants.pivotMotorIsInverted, 
      IdleMode.kCoast
    );

    rollerMotor = new WarriorSparkMax(
      Constants.GroundIntakeConstants.HardwareConstants.rollerMotorID,
      MotorType.kBrushless,
      Constants.GroundIntakeConstants.HardwareConstants.rollerMotorIsInverted,
      IdleMode.kBrake
    );

    encoder = new DutyCycleEncoder(
      Constants.GroundIntakeConstants.HardwareConstants.pivotEncoderDIO,
      Constants.GroundIntakeConstants.HardwareConstants.pivotEncoderFullRange,
      Constants.GroundIntakeConstants.HardwareConstants.pivotEncoderZero
    );

    algaeSensor = new DigitalInput(Constants.GroundIntakeConstants.HardwareConstants.digitalInputDIO);
   

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

  public void setRollerSpeed(double speed){
    rollerMotor.set(speed);
  }

  public DutyCycleEncoder getEncoder(){
    return encoder;
  }

  public DigitalInput getDigitalInput(){
    return algaeSensor;
  }



}
