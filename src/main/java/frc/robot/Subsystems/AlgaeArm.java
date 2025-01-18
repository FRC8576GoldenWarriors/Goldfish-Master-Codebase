// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;


import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;


import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drivers.WarriorSparkMax;
import frc.robot.Constants;

public class AlgaeArm extends SubsystemBase {
  /** Creates a new AlgaeArm. */
  private WarriorSparkMax armMotor;
  private DutyCycleEncoder armAbsEncoder;
  public AlgaeArm() {
    armMotor = new WarriorSparkMax(Constants.AlgaeArmConstants.ArmConstants.armMotorID, 
    MotorType.kBrushless, 
    Constants.AlgaeArmConstants.ArmConstants.motorIsInverted, 
    IdleMode.kBrake
  );

    armAbsEncoder = new DutyCycleEncoder(Constants.AlgaeArmConstants.ArmConstants.armEncoderDIO);
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setArmVoltage(double volts){
    armMotor.setVoltage(volts);
  }

  public void setArmSpeed(double speed){
    armMotor.set(speed);
  }

  
  public SparkMax getMotor(){
    return armMotor;
  }

  //radians per second
  public double getArmVelocity(){
   return getMotor().getEncoder().getVelocity()/60; //rpm/60 

  }
  
  public DutyCycleEncoder getDutyCycleEncoder(){
    return armAbsEncoder;
  }

  public double getEncoderPosition(){
    return armAbsEncoder.get();
  }

  public DutyCycleEncoder getEncoder(){
    return armAbsEncoder;
  }

  

  
}
