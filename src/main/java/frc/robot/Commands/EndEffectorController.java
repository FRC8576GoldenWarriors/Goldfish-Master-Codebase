// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.EndEffector;

public class EndEffectorController extends Command {

  private EndEffector algaePincher;

  private boolean isFinished;
  private double speed;

  public EndEffectorController(EndEffector endEffector, double speed) {
    this.algaePincher = endEffector;

    isFinished = false;
    this.speed = speed;
    addRequirements(endEffector);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    algaePincher.setSpeed(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    algaePincher.setSpeed(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
