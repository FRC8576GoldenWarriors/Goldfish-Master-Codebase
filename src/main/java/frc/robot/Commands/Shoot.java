// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import com.revrobotics.spark.SparkBase.ControlType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Shintake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Shoot extends Command {
  private Shintake shintake;
  private double upperRollerRPM;
  private double lowerRollerRPM;

  /** Creates a new Shoot. */
  public Shoot(Shintake shintake, double upperRollerRPM, double lowerRollerRPM) {
    this.shintake = shintake;
    this.upperRollerRPM = upperRollerRPM;
    this.lowerRollerRPM = lowerRollerRPM;
    // Use addRequirements() here to declare subsystem dependencies.]
    addRequirements(shintake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shintake
        .getUpperRollerMotor()
        .getClosedLoopController()
        .setReference(upperRollerRPM, ControlType.kMAXMotionVelocityControl);
    shintake
        .getLowerRollerMotor()
        .getClosedLoopController()
        .setReference(lowerRollerRPM, ControlType.kMAXMotionVelocityControl);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shintake.setIsRevved(false);
    shintake.setRollersVoltage(0);
    shintake.setRollersVoltage(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
