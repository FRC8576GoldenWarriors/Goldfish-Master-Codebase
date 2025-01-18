// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Subsystems.AlgaePincher;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlgaePincherOut extends Command {
  
  private AlgaePincher algaePincher;

  public AlgaePincherOut(AlgaePincher algaePincher) {
    this.algaePincher = algaePincher;
    addRequirements(algaePincher);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (algaePincher.hasAlgae()) {
      algaePincher.setPincherSpeed(Constants.AlgaeArmConstants.PincherConstants.pincherOutSpeed);
    }
    else {
      new WaitCommand(0.25); // time needs to be adjusted so it works
      algaePincher.setPincherSpeed(0);
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
