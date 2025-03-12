// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.GroundIntake;
import org.littletonrobotics.junction.Logger;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class GroundIntakeController extends Command {

  private GroundIntake intake;

  private DutyCycleEncoder encoder;

  private double desiredAngle;
  private double rollerSpeed;

  private double PIDvoltage;
  private double FFvoltage;
  private double voltage;
  
  private double pivotPositon;

  private TrapezoidProfile.Constraints constraints;
  private ProfiledPIDController pid;

  private ArmFeedforward feedforward;


  public GroundIntakeController(GroundIntake intake, double desiredAngle, double rollerSpeed) {
    this.intake = intake;
    this.encoder = intake.getEncoder();
    this.rollerSpeed = rollerSpeed;

    this.desiredAngle = desiredAngle;

    constraints = new TrapezoidProfile.Constraints(3.0, 5.0);
    pid =
        new ProfiledPIDController(
            Constants.GroundIntakeConstants.ControlConstants.kP,
            Constants.GroundIntakeConstants.ControlConstants.kI,
            Constants.GroundIntakeConstants.ControlConstants.kD,
            constraints);
    feedforward = new ArmFeedforward(Constants.GroundIntakeConstants.ControlConstants.kS, Constants.GroundIntakeConstants.ControlConstants.kG, Constants.GroundIntakeConstants.ControlConstants.kV);
    
    pivotPositon = 0.0;
    PIDvoltage = 0.0;
    FFvoltage = 0.0;
    voltage = 0.0;

    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // if (encoder.get() - desiredAngle < 0) { // going down
    //   motorOutput = 0.4;
    //   SmartDashboard.putBoolean("True: Going Up, False: Going Down", false);
    //   Logger.recordOutput("Ground_Intake/Ground Intake Up", false);
    //   if (Math.abs(encoder.get() - desiredAngle) < 0.03) {
    //     motorOutput = 0.0;
    //     angleReached = true;
    //   }
    // } else { // going up
    //   motorOutput = -0.4;
    //   SmartDashboard.putBoolean("True: Going Up, False: Going Down", true);
    //   Logger.recordOutput("Ground_Intake/Ground Intake Up", true);
    //   if (Math.abs(encoder.get() - desiredAngle) < 0.03) {
    //     motorOutput = 0.0;
    //     angleReached = true;
    //   }
    // }

    // if(angleReached){
    //   motorOutput = 0.0;
    // }

    // intake.setPivotSpeed(motorOutput);

    pivotPositon = intake.getEncoderPosition();

    FFvoltage = feedforward.calculate((pivotPositon+0.25)*Math.PI*2, 1.0); //position in radians, 0 is horizontal
    PIDvoltage = pid.calculate(intake.getEncoderPosition(), desiredAngle);

    voltage = FFvoltage + PIDvoltage;

    intake.setPivotVoltage(0.0); // CHANGE AFTER INTAKE IS REATTAHCHED
    intake.setRollerSpeed(rollerSpeed);

    SmartDashboard.putNumber("Intake Pivot Voltage Output", voltage);
    Logger.recordOutput("Ground_Intake/Intake Pivot Voltage Output", voltage);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setPivotSpeed(0);
    intake.setRollerSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
