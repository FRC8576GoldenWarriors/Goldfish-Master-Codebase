package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimbMech;

public class WindRope extends CommandBase {
  private final ClimbMech climbMech;

  public WindRope(ClimbMech climbMech) {
    this.climbMech = climbMech;
    addRequirements(climbMech);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    climbMech.windRope();
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