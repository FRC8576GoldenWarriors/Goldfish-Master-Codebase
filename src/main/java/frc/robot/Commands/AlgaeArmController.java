// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.AlgaeArm;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlgaeArmController extends Command {

  private AlgaeArm algaeArm;

  private DutyCycleEncoder encoder;

  private ArmFeedforward feedForward;
  private PIDController feedback;

  private double setpoint;
  private double voltage;


  public AlgaeArmController(AlgaeArm algaeArm, double setpoint) {
    
    this.algaeArm = algaeArm;

    this.feedForward = new ArmFeedforward(
      Constants.AlgaeArmConstants.ArmConstants.kS, 
      Constants.AlgaeArmConstants.ArmConstants.kG,
      Constants.AlgaeArmConstants.ArmConstants.kV, 
      Constants.AlgaeArmConstants.ArmConstants.kA
    );

    this.feedback = new PIDController(
      Constants.AlgaeArmConstants.ArmConstants.kP, 
      Constants.AlgaeArmConstants.ArmConstants.kI, 
      Constants.AlgaeArmConstants.ArmConstants.kD
    );


    this.setpoint = setpoint;

    this.encoder = algaeArm.getEncoder();


    addRequirements(algaeArm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    voltage = feedForward.calculate(encoder.get(), algaeArm.getArmVelocity()) + feedback.calculate(encoder.get(), setpoint);
    algaeArm.setArmVoltage(voltage);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    algaeArm.setArmVoltage(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return encoder.get()>Constants.AlgaeArmConstants.ArmConstants.highSoftStopPosition
    || encoder.get()<Constants.AlgaeArmConstants.ArmConstants.lowSoftStopPositon; //make constant later
  }
}
