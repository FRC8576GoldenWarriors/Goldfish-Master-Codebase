// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.Arm;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ArmController extends Command {

  private Arm arm;

  private DutyCycleEncoder encoder;

  private ArmFeedforward feedForward;
  private PIDController feedback;

  private double setpoint;

  private double FFVoltage;
  private double PIDVoltage;
  private double voltage;

  private double COMOffset;

  public ArmController(Arm arm, double setpoint) {

    this.arm = arm;

    this.feedForward =
        new ArmFeedforward(
            Constants.ArmConstants.ControlConstants.kS,
            Constants.ArmConstants.ControlConstants.kG,
            Constants.ArmConstants.ControlConstants.kV,
            Constants.ArmConstants.ControlConstants.kA);

    this.feedback =
        new PIDController(
            Constants.ArmConstants.ControlConstants.kP,
            Constants.ArmConstants.ControlConstants.kI,
            Constants.ArmConstants.ControlConstants.kD);

    this.setpoint = setpoint;

    this.encoder = arm.getEncoder();

    this.COMOffset = 0.013194;

    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    FFVoltage = feedForward.calculate(setpoint + COMOffset, arm.getArmVelocity());

    PIDVoltage = feedback.calculate(encoder.get(), setpoint);

    voltage = FFVoltage + PIDVoltage;

    SmartDashboard.putNumber("Arm FF Voltage", FFVoltage);
    SmartDashboard.putNumber("Arm PID Voltage", PIDVoltage);

    if (encoder.get() < 0.02 || encoder.get() > 0.6) {
      voltage = 0.0;
      arm.setArmSpeed(0);
      arm.setArmMotorIdleMode(IdleMode.kBrake);
    }

    arm.setArmVoltage(voltage);
    SmartDashboard.putNumber("Arm Total Controller Voltage", voltage);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.setArmVoltage(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;

    // return encoder.get() > Constants.ArmConstants.ControlConstants.highSoftStopPosition
    //     || encoder.get() < Constants.ArmConstants.ControlConstants.lowSoftStopPositon; // make
    // constant later
  }
}
