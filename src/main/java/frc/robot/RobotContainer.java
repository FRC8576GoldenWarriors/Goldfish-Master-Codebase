// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
//import frc.robot.Commands.AdjustRobotPos;
import frc.robot.Commands.SwerveDrive;
import frc.robot.Constants.Constants;
//import frc.robot.Subsystems.AprilTagStats;
import frc.robot.Subsystems.Drivetrain;


public class RobotContainer {
  public static final Drivetrain m_drivetrain = Drivetrain.getInstance();
 // public static final AprilTagStats m_aprilTagStats = new AprilTagStats(Constants.VisionConstants.nameConstants.cameraName,
 // Constants.VisionConstants.nameConstants.publishName,
  //Constants.VisionConstants.nameConstants.tabName);

  public static final CommandXboxController driverController = new CommandXboxController(Constants.ControllerConstants.driverControllerPort);
  CommandXboxController operatorController = new CommandXboxController(Constants.ControllerConstants.operatorControllerPort);

  private final JoystickButton resetHeading_Start = new JoystickButton(driverController.getHID(), XboxController.Button.kStart.value);

  private final SendableChooser<Command> autoChooser;


  public RobotContainer() {
    m_drivetrain.setDefaultCommand(new SwerveDrive());
   //Add all the choise of Autonomous modes to the Smart Dashboard
    autoChooser = AutoBuilder.buildAutoChooser();
    
    configureBindings();
    SmartDashboard.putData("Auto Chooser",autoChooser);
  }

  private void configureBindings() {
    //Driver controller
    resetHeading_Start.onTrue(
      new InstantCommand(m_drivetrain::zeroHeading, m_drivetrain));
   // driverController.a().onTrue(new AdjustRobotPos(m_drivetrain, m_aprilTagStats, 15));
   // driverController.a().and(driverController.b()).onTrue(new AdjustRobotPos(m_drivetrain, m_aprilTagStats, -15));
    // operatorController.y().whileTrue(new ElevatorUpCommand(m_elevator));
    // operatorController.a().whileTrue(new ElevatorDownCommand(m_elevator));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public void registerNamedCommands(){
    NamedCommands.registerCommand("Reset Swerve Encoders", new InstantCommand(()->m_drivetrain.resetAllEncoders()).
    withDeadline(new InstantCommand(()-> new WaitCommand(0.1))));

    NamedCommands.registerCommand("Reset Heading", new InstantCommand(()->m_drivetrain.zeroHeading()).
    withDeadline(new InstantCommand(()-> new WaitCommand(0.1))));

    NamedCommands.registerCommand("Auton Reset", new InstantCommand(()->m_drivetrain.autonReset()).
    withDeadline(new InstantCommand(()->new WaitCommand(0.1))));
  }
}
