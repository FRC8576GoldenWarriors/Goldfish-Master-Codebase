// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Subsystems.Simulation.DrivetrainSim;
import frc.robot.Subsystems.Simulation.SimConstants;

public class SimSwerveDrive extends Command {
  private DrivetrainSim m_drivetrainSim = DrivetrainSim.getInstance();

  /** Creates a new SwerveDrive. */
  public SimSwerveDrive() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drivetrainSim);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.m_drivetrainSim.swerveDrive(
        -RobotContainer.driverController.getLeftY()
            * Math.abs(RobotContainer.driverController.getLeftY())
            * SimConstants.DriverConstants.xCoefficient, // 2.25
        -RobotContainer.driverController.getLeftX()
            * Math.abs(RobotContainer.driverController.getLeftX())
            * SimConstants.DriverConstants.yCoefficient, // 2.25
        -RobotContainer.driverController.getRightX()
            * Math.abs(RobotContainer.driverController.getRightX())
            * SimConstants.DriverConstants.turnCoefficient, // 1.75
        true, // !RobotContainer.driverController.getHID().getRawButton(XboxController.Button.kB.value)
        new Translation2d(),
        true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_drivetrainSim.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
