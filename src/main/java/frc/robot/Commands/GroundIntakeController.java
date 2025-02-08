// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.GroundIntake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class GroundIntakeController extends Command {
  /** Creates a new GroundIntakeController. */
  private GroundIntake intake;

  private PIDController pid;

  private double setAngle;

  public GroundIntakeController(GroundIntake intake, double setAngle) {
    this.intake = intake;
    this.setAngle = setAngle;

    pid =
        new PIDController(
            Constants.GroundIntakeConstants.ControlConstants.kP,
            Constants.GroundIntakeConstants.ControlConstants.kI,
            Constants.GroundIntakeConstants.ControlConstants.kD);

    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    intake.setPivotSpeed(pid.calculate(intake.getEncoderPosition(), setAngle));
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
