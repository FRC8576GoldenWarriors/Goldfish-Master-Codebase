package frc.robot.Commands;

import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.Climber;

public class ClimbUp extends Command {
  private final Climber climber;
  private RelativeEncoder encoder;

  private final PIDController pid;

  public ClimbUp(Climber climber) {
    this.climber = climber;
    encoder = climber.getRelativeEncoder();
    pid = new PIDController(0, 0, 0);

    addRequirements(climber);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    climber.setMotorVoltage(
        pid.calculate(encoder.getPosition(), Constants.ClimberConstants.climbPosition));
  }

  @Override
  public void end(boolean interrupted) {
    climber.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
