package frc.robot.Commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.Climber;

public class Climb extends Command {
  private final Climber climbMech;
  private boolean isClimbUp;
  private double FFVal;
  private double PIDVal;

  public Climb(Climber climbMech, boolean isClimbUp) {
    this.climbMech = climbMech;
    this.isClimbUp = isClimbUp;
    addRequirements(climbMech);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (isClimbUp) {

      PIDVal =
          Constants.ClimberConstants.ControlConstants.windPID.calculate(
              Constants.ClimberConstants.ControlConstants.windingSpeed);
      SmartDashboard.putNumber("Climb PID", PIDVal);

      FFVal =
          Constants.ClimberConstants.ControlConstants.climbFeedforward.calculate(
              Constants.ClimberConstants.ControlConstants.unwindingSpeed);
      SmartDashboard.putNumber("Climb FeedForward", FFVal);
      climbMech.climbing(true);
      climbMech.windRope(PIDVal + FFVal);

    } else {
      PIDVal =
          Constants.ClimberConstants.ControlConstants.unwindPID.calculate(
              Constants.ClimberConstants.ControlConstants.unwindingSpeed);
      SmartDashboard.putNumber("Climb PID", FFVal);

      FFVal =
          Constants.ClimberConstants.ControlConstants.climbFeedforward.calculate(
              Constants.ClimberConstants.ControlConstants.unwindingSpeed);
      SmartDashboard.putNumber("Climb FeedForward", FFVal);
      climbMech.windRope(PIDVal + FFVal);
    }
  }

  @Override
  public void end(boolean interrupted) {
    climbMech.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
