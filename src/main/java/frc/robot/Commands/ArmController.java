// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.Arm;
import org.littletonrobotics.junction.Logger;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ArmController extends Command {

  private Arm arm;

  private DutyCycleEncoder encoder;

  private ArmFeedforward feedForward;
  private PIDController feedback;

  private TrapezoidProfile.Constraints constraints;
  //private ProfiledPIDController feedback;


  private double setpoint;
  private double voltage;

  private double FFVoltage;
  private double PIDVoltage;

  private boolean isFinished;
  private double COMOffset;

  public ArmController(Arm arm, double setpoint) {

    this.arm = arm;

    constraints = new TrapezoidProfile.Constraints(3.0, 3.5);

    

    feedback =
    //     new ProfiledPIDController(
    //         Constants.ArmConstants.ControlConstants.kP,
    //         Constants.ArmConstants.ControlConstants.kI,
    //         Constants.ArmConstants.ControlConstants.kD,
    //         constraints);

    new PIDController(
        Constants.ArmConstants.ControlConstants.kP,
        Constants.ArmConstants.ControlConstants.kI,
        Constants.ArmConstants.ControlConstants.kD);

    feedForward =
        new ArmFeedforward(
            Constants.ArmConstants.ControlConstants.kS,
            Constants.ArmConstants.ControlConstants.kG,
            Constants.ArmConstants.ControlConstants.kV,
            Constants.ArmConstants.ControlConstants.kA);

    this.setpoint = setpoint;

    encoder = arm.getEncoder();

    COMOffset = 0.013194;

    // feedback.reset(arm.getEncoderPosition());

    // feedback.setGoal(setpoint);

    isFinished = false;

    addRequirements(arm);

    // SmartDashboard.putNumber("Arm Kg", Constants.ArmConstants.ControlConstants.kG);
  }



  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // double kg =0;
    // SmartDashboard.getNumber("Arm kg", kg);

    // feedForward.setKg(kg);

    // ff passed in radians
    FFVoltage = feedForward.calculate(((setpoint + COMOffset - 0.25) * Math.PI * 2), 2.0);

    PIDVoltage = feedback.calculate(encoder.get(), setpoint);



    voltage = FFVoltage + PIDVoltage;

    SmartDashboard.putNumber("Arm FF Voltage", FFVoltage);
    SmartDashboard.putNumber("Arm PID Voltage", PIDVoltage);
    SmartDashboard.putNumber("Arm Total Voltage", voltage);
    Logger.recordOutput("Arm/FF Voltage", FFVoltage);
    Logger.recordOutput("Arm/PID Voltage", PIDVoltage);

    if (encoder.get() > 0.78) { // 0.725) {
      voltage = 0.0;
    }

    if (!encoder.isConnected()) {
      voltage = 0.0;
    }

    arm.setArmVoltage(voltage);
    SmartDashboard.putNumber("Arm/Total Controller Voltage", voltage);
    Logger.recordOutput("Arm/Total Controller Voltage", voltage);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.setArmVoltage(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
