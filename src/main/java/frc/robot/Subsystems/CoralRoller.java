// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class CoralRoller extends SubsystemBase {
    
    private SparkMax coralRoller;
    private DigitalInput intakeSensor;

    public CoralRoller() {
        coralRoller = new SparkMax(Constants.IntakeConstants.coralIntakeID, MotorType.kBrushless);
        intakeSensor = new DigitalInput(Constants.IntakeConstants.coralIntakeDigiSensorID);
    }

    public void setRollerSpeed(double speed){
        coralRoller.set(speed);
    }
    public DigitalInput getDigitalInput(){
        return intakeSensor;
    }
    public boolean hasCoral() {
        return intakeSensor.get();
    }
    
    @Override
    public void periodic() {
    }
}