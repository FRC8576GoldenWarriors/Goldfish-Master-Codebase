// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Shintake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Shoot extends Command {
  private Shintake shintake;
  private double topRollerSpeed;
  private double bottomRollerSpeed;

  /** Creates a new Shoot. */
  public Shoot(Shintake shintake, double topRollerSpeed, double bottomRollerSpeed) {
    this.shintake = shintake;
    this.topRollerSpeed = topRollerSpeed;
    this.bottomRollerSpeed = bottomRollerSpeed;
    // Use addRequirements() here to declare subsystem dependencies.]
    addRequirements(shintake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shintake.setLowerRollerSpeed(bottomRollerSpeed);
    shintake.setUpperRollerSpeed(topRollerSpeed);
    if (shintake.getAverageEncoderVelocity() == (bottomRollerSpeed + topRollerSpeed) / 2)
      shintake.setIsRevved(true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shintake.setIsRevved(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
