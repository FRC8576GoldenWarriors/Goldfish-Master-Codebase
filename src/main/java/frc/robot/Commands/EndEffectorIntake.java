// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.EndEffector;

public class EndEffectorIntake extends Command {

  private EndEffector algaePincher;

  private boolean isFinished;
  private double speed;

  public EndEffectorIntake(EndEffector algaePincher, double speed) {
    this.algaePincher = algaePincher;

    isFinished = false;
    this.speed = speed;
    addRequirements(algaePincher);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    // if (!algaePincher.hasAlgae()) {
    //   algaePincher.setSpeed(Constants.EndEffectorConstants.pincherInSpeed);
    // } else {
    //   isFinished = true;
    // }
    algaePincher.setSpeed(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    // new WaitCommand(Constants.EndEffectorConstants.pincherInRunExtension);

    algaePincher.setSpeed(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
