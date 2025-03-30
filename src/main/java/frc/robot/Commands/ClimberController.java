package frc.robot.Commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.Climber;
import org.littletonrobotics.junction.Logger;

public class ClimberController extends Command {
  private final Climber climber;

  private double bangBangVoltage;
  private double climbAngle;
  private double motorOutput;

  public ClimberController(Climber climber, double climbAngle) {
    this.climber = climber;

    this.climbAngle = climbAngle;

    addRequirements(climber);

    if (this.climbAngle == Constants.ClimberConstants.ControlConstants.climberUpPosition) {
      this.climber.setClimbing(true);
    }
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (climber.getEncoderPosition() < climbAngle) { // pivot going down, robot climbing up
      motorOutput = 1.0;

      Logger.recordOutput("Climber/Climbing Up", true);
      if (Math.abs(climber.getEncoderPosition() - climbAngle) < 0.005) {
        motorOutput = 0.0;
      }
    } else { // pivot going up, robot going down
      motorOutput = -1.0;
      Logger.recordOutput("Climber/Climbing Up", false);
      if (Math.abs(climber.getEncoderPosition() - climbAngle) < 0.005) {
        motorOutput = 0.0;
      }
    }

    if (climber.getEncoderPosition() > 0.20) {
      motorOutput = 0.0;
    }

    climber.setMotorSpeed(motorOutput);

    SmartDashboard.putNumber("Climb Motor Encoder", climber.getEncoderPosition());

    Logger.recordOutput("Climber/Climb Bang Bang Voltage", bangBangVoltage);
    Logger.recordOutput("Climber/Climb Total Voltage", motorOutput);
    Logger.recordOutput("Climber/Climb Encoder Position", climber.getEncoderPosition());
  }

  @Override
  public void end(boolean interrupted) {
    climber.setMotorSpeed(0.0);
    climber.setClimbing(false);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
