// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.Intake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeFFController extends Command {
  private ArmFeedforward armFF;
  private PIDController armPID;
  private double goalEndPoint;//Radians
  private Intake intake;
  /** Creates a new IntakeFFController. */
  public IntakeFFController(double goalPoint,Intake intake){
  armFF = new ArmFeedforward(Constants.IntakeConstants.ControlConstants.kS,
    Constants.IntakeConstants.ControlConstants.kG, 
    Constants.IntakeConstants.ControlConstants.kV);

    armPID = new PIDController(Constants.IntakeConstants.ControlConstants.kP,
    Constants.IntakeConstants.ControlConstants.kI,
    Constants.IntakeConstants.ControlConstants.kD);
    

    goalEndPoint = Units.degreesToRadians(goalPoint);//Degrees to radians

    armPID.setIZone(goalEndPoint/1.3);
    this.intake = intake;

    addRequirements(intake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    

    double voltagePID = armPID.calculate(intake.getAbsoluteEncoderPosition(),goalEndPoint);

    double voltageFF = armFF.calculate(goalEndPoint,
    Constants.IntakeConstants.HardwareConstants.kMaxArmVelocity,
    Constants.IntakeConstants.HardwareConstants.kMaxArmAcceleration);

    double voltage = voltagePID + voltageFF;

    if(intake.getAbsoluteEncoderPosition()>=3.114 && voltage>0){
      voltage = 0;
    };

    SmartDashboard.putNumber("Intake FF Volts", voltageFF);
    SmartDashboard.putNumber("Intake PID Volts", voltagePID);

    intake.setVoltage(voltage);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setVoltage(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false; 
  }
}
