// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Simulation.SimConstants;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private Drivetrain m_drivetrain = Drivetrain.getInstance();
  private RobotContainer m_robotContainer;

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("Time", DriverStation.getMatchTime());
    SmartDashboard.putNumber("Voltage", RobotController.getBatteryVoltage());
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    m_drivetrain.resetAllEncoders();
    m_drivetrain.setAllIdleMode(true);

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }

    // m_drivetrain.zeroHeading();
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    // m_drivetrain.setHeading((m_drivetrain.getHeading()+180));
    if (SimConstants.currentMode.equals(SimConstants.Mode.REAL)) {
      m_drivetrain.resetAllEncoders();
      m_drivetrain.setAllIdleMode(true);
    }
  }

  @Override
  public void teleopPeriodic() {
    if (SimConstants.currentMode.equals(SimConstants.Mode.REAL)) {
      SmartDashboard.putNumber(
          "Arm Encoder Abs. Rotation", RobotContainer.m_arm.getEncoderPosition());
      SmartDashboard.putNumber(
          "Ground Intake Abs. Rotation", RobotContainer.m_groundIntake.getEncoderPosition());
    }
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
