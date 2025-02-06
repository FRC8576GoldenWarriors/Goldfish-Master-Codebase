// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.net.URI;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Constants.*;

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  private Drivetrain m_drivetrain = Drivetrain.getInstance();
  private RobotContainer m_robotContainer;


  public Robot(){
    


    switch (Constants.SimulationConstants.currentMode){
      case REAL:
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;

        case SIMULATION:
          // Run a physics simulator, log it NT
          Logger.addDataReceiver(new NT4Publisher());
          break;

          case REPLAY:
            // Replaying a log, set up the replay source
            setUseTiming(false);
            String logPath = LogFileUtil.findReplayLog();
            Logger.setReplaySource(new WPILOGReader(logPath));
            Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
            break;
    }

    // Initilize URCL
    //Logger.registerURCL(URCL.startExternal());

    Logger.start();

    m_robotContainer = new RobotContainer();
  }
 
  
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

    m_drivetrain.zeroHeading();
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

    m_drivetrain.resetAllEncoders();
    m_drivetrain.setAllIdleMode(true);
  }

  @Override
  public void teleopPeriodic() {}

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

  @Override
  public void simulationPeriodic() {}
}
