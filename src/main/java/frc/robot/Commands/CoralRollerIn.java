// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.CoralRoller;

public class CoralRollerIn extends Command {

  private CoralRoller coralRoller;

  /** Creates a new CoralRollerIn. */
  public CoralRollerIn(CoralRoller coralRoller) {
    this.coralRoller = coralRoller;
    addRequirements(coralRoller);
  }

  @Override
  public void initialize() {
    //dunno what to put here since we don't have an encoder associated with this
  }

  @Override
  public void execute() {
    if (!coralRoller.hasCoral()) {
      coralRoller.setRollerSpeed(Constants.IntakeConstants.coralIntakeInSpeed);
    }
    else {
      coralRoller.setRollerSpeed(0);
    }
  }

  @Override
  public void end(boolean interrupted) {
    coralRoller.setRollerSpeed(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
