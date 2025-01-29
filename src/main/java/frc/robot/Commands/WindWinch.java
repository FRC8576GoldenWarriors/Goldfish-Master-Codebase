package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.ClimbMech;


public class WindWinch extends Command {
  private final ClimbMech climbMech;

  public WindWinch(ClimbMech climbMech) {
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