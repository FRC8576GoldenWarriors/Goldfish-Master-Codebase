package frc.robot.Commands;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.Climber;
import org.littletonrobotics.junction.Logger;

public class Climb extends Command {
  private final Climber climber;

  private final ElevatorFeedforward feedforward;

  private final BangBangController bangBangController;

  private double bangBangVoltage;
  private double FFVoltage;
  private double outputVoltage;

  public Climb(Climber climber) {
    this.climber = climber;

    this.feedforward =
        new ElevatorFeedforward(
            Constants.ClimberConstants.ControlConstants.kS,
            Constants.ClimberConstants.ControlConstants.kG,
            Constants.ClimberConstants.ControlConstants.kV);

    this.bangBangController = new BangBangController();

    addRequirements(climber);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    bangBangVoltage =
        bangBangController.calculate(
            climber.getEncoderPosition(),
            Constants.ClimberConstants.ControlConstants.climberUpPosition);

    FFVoltage = feedforward.calculate(0);

    outputVoltage = bangBangVoltage + FFVoltage;

    climber.setMotorVoltage(outputVoltage);

    SmartDashboard.putNumber("Climb Motor Encoder", climber.getEncoderPosition());

    Logger.recordOutput("Climber/Climb Bang Bang Voltage", bangBangVoltage);
    Logger.recordOutput("Climber/Climb FF Voltage", FFVoltage);
    Logger.recordOutput("Climber/Climb Total Voltage", outputVoltage);
    Logger.recordOutput("Climber/Climb Encoder Position", climber.getEncoderPosition());
  }

  @Override
  public void end(boolean interrupted) {
    climber.setMotorVoltage(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
