// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import com.revrobotics.spark.SparkBase.ControlType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Shintake;

public class ShootRPM extends Command {
  private Shintake shintake;
  private double upperRollerRPM;
  private double lowerRollerRPM;

 
  public ShootRPM(Shintake shintake, double upperRollerRPM, double lowerRollerRPM) {
    this.shintake = shintake;
    this.upperRollerRPM = upperRollerRPM;
    this.lowerRollerRPM = lowerRollerRPM;
    
    addRequirements(shintake);
  }


  @Override
  public void initialize() {}

  @Override
  public void execute() {
    shintake
        .getUpperRollerMotor()
        .getClosedLoopController()
        .setReference(upperRollerRPM, ControlType.kMAXMotionVelocityControl);
    shintake
        .getLowerRollerMotor()
        .getClosedLoopController()
        .setReference(lowerRollerRPM, ControlType.kMAXMotionVelocityControl);
  }


  @Override
  public void end(boolean interrupted) {
    shintake.setIsRevved(false);
    shintake.setRollersVoltage(0);
    shintake.setRollersVoltage(0);
  }


  @Override
  public boolean isFinished() {
    return false;
  }
}
