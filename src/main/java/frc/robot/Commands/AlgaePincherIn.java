// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Subsystems.AlgaePincher;

public class AlgaePincherIn extends Command {

  private AlgaePincher algaePincher;

  private boolean isFinished;

  public AlgaePincherIn(AlgaePincher algaePincher) {
    this.algaePincher = algaePincher;

    isFinished = false;

    addRequirements(algaePincher);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (!algaePincher.hasAlgae()) {
      algaePincher.setPincherSpeed(Constants.AlgaeArmConstants.PincherConstants.pincherInSpeed);
    } else {
      isFinished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    new WaitCommand(0.25);

    algaePincher.setPincherSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
