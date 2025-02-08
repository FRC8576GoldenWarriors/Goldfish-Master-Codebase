// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Commands.IntakeFFController;
import frc.robot.Commands.SwerveDrive;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Intake;

public class RobotContainer {
  public final JoystickButton resetHeading_Start =
      new JoystickButton(driverController.getHID(), XboxController.Button.kStart.value);
  public static final Intake m_intake = new Intake();
  public static final CommandXboxController driverController =
      new CommandXboxController(0);
  public static final Drivetrain m_drivetrain = new Drivetrain();
  public RobotContainer() {
    m_drivetrain.setDefaultCommand(new SwerveDrive());
    configureBindings();
  }

  private void configureBindings() {
    resetHeading_Start.onTrue(new InstantCommand(m_drivetrain::zeroHeading, m_drivetrain));
    driverController.a().onTrue(new IntakeFFController(90, m_intake));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
