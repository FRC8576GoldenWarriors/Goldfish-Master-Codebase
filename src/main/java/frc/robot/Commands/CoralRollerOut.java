// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Subsystems.CoralRoller;

public class CoralRollerOut extends Command {

  private CoralRoller coralRoller;

  private boolean isFinished;

  public CoralRollerOut(CoralRoller coralRoller) {
    this.coralRoller = coralRoller;
    
    isFinished = false;

    addRequirements(coralRoller);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //dunno what to put here since we don't have an encoder associated with this
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (coralRoller.hasCoral()) {
      coralRoller.setRollerSpeed(Constants.CoralRollerConstants.coralIntakeOutSpeed);
    }
    else {
      isFinished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    new WaitCommand(0.25);

    coralRoller.setRollerSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
