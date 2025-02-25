// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.GroundIntake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class GroundIntakeController extends Command {

  GroundIntake intake;
  PIDController pid;

  DutyCycleEncoder encoder;

  double desiredAngle;
  double motorOutput;
  double rollerSpeed;

  boolean isFinished;

  public GroundIntakeController(GroundIntake intake, double desiredAngle, double rollerSpeed) {
    this.intake = intake;
    this.encoder = intake.getEncoder();
    this.rollerSpeed = rollerSpeed;

    this.desiredAngle = desiredAngle;

    pid = new PIDController(1.0, 0.0, 0.0);

    addRequirements(intake);
  
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {


   

    if (encoder.get() - desiredAngle < 0) { // going down
      motorOutput = 0.3;
      SmartDashboard.putBoolean("True: Going Up, False: Going Down", false);
      if (Math.abs(encoder.get() - desiredAngle) < 0.025) {
        motorOutput = 0.0;
      }
    } else { // going up
      motorOutput = -0.3;
      SmartDashboard.putBoolean("True: Going Up, False: Going Down", true);
      if (Math.abs(encoder.get() - desiredAngle) < 0.025) {
        motorOutput = 0.0;
      }
    }

    intake.setPivotSpeed(motorOutput);
    intake.setRollerSpeed(rollerSpeed);
    
    SmartDashboard.putNumber("Intake Pivot Motor Output", motorOutput);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setPivotSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
