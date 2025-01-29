// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Subsystems.EndEffector;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlgaePincherOut extends Command {

  private EndEffector endEffector;

  private boolean isFinished;

  public AlgaePincherOut(EndEffector endEffector) {
    this.endEffector = endEffector;

    isFinished = false;

    addRequirements(endEffector);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (endEffector.hasAlgae()) {
      endEffector.setPincherSpeed(Constants.AlgaeArmConstants.PincherConstants.pincherOutSpeed);
    } else {
      isFinished = true;
    }
  }

  @Override
  public void end(boolean interrupted) {

    new WaitCommand(0.25);
    endEffector.setPincherSpeed(0);
  }

  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
