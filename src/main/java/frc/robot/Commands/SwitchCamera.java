// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.DriverStream;

public class SwitchCamera extends Command {

  private DriverStream driverStream;

  public SwitchCamera(DriverStream driverStream) {

    // this.driverStream = driverStream;
    // addRequirements(driverStream);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    driverStream.nextStream();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
