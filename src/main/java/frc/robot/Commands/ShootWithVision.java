// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Subsystems.Shintake;

public class ShootWithVision extends Command {
  Shintake shintake;
  public ShootWithVision(Shintake shintake) {
    this.shintake = shintake;

    addRequirements(shintake);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    double t = Math.sqrt( (2 * Constants.ShooterConstants.accelerationY) / Constants.ShooterConstants.gravity );
    double changeX = 0;
    double initialVelocityX = (changeX - (0.5 * Constants.ShooterConstants.accelerationX * Math.pow(t,2) ) ) / t;
    double initialVelocityY = (Constants.ShooterConstants.changeY - (0.5 * Constants.ShooterConstants.gravity * Math.pow(t,2) ) ) / t;
    double resultedVelocity = Math.sqrt( Math.pow(initialVelocityX, 2) + Math.pow(initialVelocityY, 2) );
    double velocityWheels = resultedVelocity; // Vi - Vdrivetrain
    double velocityDrivetrain;

    double desiredRPM = velocityWheels * 60 / (Math.PI * Constants.SwerveConstants.WHEEL_DIAMETER);
    double voltage = shintake.calculateMotorVoltage(desiredRPM);
    shintake.setRollersVoltage(voltage);
  }

  @Override
  public void end(boolean interrupted) {
    shintake.setLowerRollerSpeed(0);
    shintake.setUpperRollerSpeed(0);
  }

  @Override
  public boolean isFinished() {
    return !RobotContainer.m_groundIntake.getDigitalInput().get();
  }
}
