// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.util.PixelFormat;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
//import frc.robot.Commands.AdjustRobotPos;
import frc.robot.Commands.SwerveDrive;
//import frc.robot.Subsystems.AprilTagStats;
import frc.robot.Subsystems.Drivetrain;


public class RobotContainer {

  public static final Drivetrain m_drivetrain = Drivetrain.getInstance();

  public static final CommandXboxController driverController = new CommandXboxController(Constants.ControllerConstants.driverControllerPort);
  public static final CommandXboxController operatorController = new CommandXboxController(Constants.ControllerConstants.operatorControllerPort);

  private final JoystickButton resetHeading_Start = new JoystickButton(driverController.getHID(), XboxController.Button.kStart.value);

  public final SendableChooser<Command> autoChooser;

 // public final UsbCamera camera;


  public RobotContainer() {
    m_drivetrain.setDefaultCommand(new SwerveDrive());
   //Add all the choise of Autonomous modes to the Smart Dashboard
    autoChooser = AutoBuilder.buildAutoChooser();

    // camera = CameraServer.startAutomaticCapture(0);
    // camera.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
    // camera.setVideoMode(PixelFormat.kMJPEG, 400, 400, 40);

    
    
    
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
