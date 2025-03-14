// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.GroundIntake;
import org.littletonrobotics.junction.Logger;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class GroundIntakeController extends Command {

  private GroundIntake intake;

  private double desiredAngle;
  private double rollerSpeed;

  private double PIDvoltage;
  private double FFvoltage;
  private double voltage;

  private double COMOffset;

  private TrapezoidProfile.Constraints constraints;
  private ProfiledPIDController pid;

  private ArmFeedforward feedforward;

  public GroundIntakeController(GroundIntake intake, double desiredAngle, double rollerSpeed) {
    this.intake = intake;

    this.rollerSpeed = rollerSpeed;

    this.desiredAngle = desiredAngle;

    constraints = new TrapezoidProfile.Constraints(3.0, 5.0);
    pid =
        new ProfiledPIDController(
            Constants.GroundIntakeConstants.ControlConstants.kP,
            Constants.GroundIntakeConstants.ControlConstants.kI,
            Constants.GroundIntakeConstants.ControlConstants.kD,
            constraints);
    feedforward =
        new ArmFeedforward(
            Constants.GroundIntakeConstants.ControlConstants.kS,
            Constants.GroundIntakeConstants.ControlConstants.kG,
            Constants.GroundIntakeConstants.ControlConstants.kV);

    PIDvoltage = 0.0;
    FFvoltage = 0.0;
    voltage = 0.0;

    COMOffset = 0.0;

    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    FFvoltage =
        feedforward.calculate(
            (-desiredAngle + 0.25 + COMOffset) * Math.PI * 2,
            1.0); // position in radians, 0 is horizontal
    PIDvoltage = -pid.calculate(intake.getEncoderPosition(), desiredAngle);

    voltage = FFvoltage + PIDvoltage;

    intake.setPivotVoltage(voltage);
    intake.setRollerSpeed(rollerSpeed);

    // if desired angle in coast zone, set to coast voltage
    // if(desiredAngle>Constants.GroundIntakeConstants.ControlConstants.coastZone &&
    // intake.getEncoderPosition()>=Constants.GroundIntakeConstants.ControlConstants.coastZone){
    //   intake.setPivotVoltage(0.0);
    // }
    // else{
    //   intake.setPivotVoltage(voltage);
    // }

    // if (rollerSpeed == 0.0) {
    //   intake.setRollerVoltage(Constants.GroundIntakeConstants.ControlConstants.rollerIdlekS);
    // } else {
    //   intake.setRollerSpeed(rollerSpeed);
    // }

    SmartDashboard.putNumber("Intake Pivot Voltage Output", voltage);
    SmartDashboard.putNumber("Intake Pivot FF", FFvoltage);
    SmartDashboard.putNumber("Intake Pivot PID", PIDvoltage);
    Logger.recordOutput("Ground_Intake/Intake Pivot Voltage Output", voltage);
    Logger.recordOutput("Ground_Intake/Intake Pivot PID", PIDvoltage);
    Logger.recordOutput("Ground_Intake/Intake Pivot FF", FFvoltage);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setPivotVoltage(0); // maybe set ff voltage?
    intake.setRollerVoltage(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
