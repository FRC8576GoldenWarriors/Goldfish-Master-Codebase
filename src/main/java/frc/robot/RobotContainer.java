package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Commands.EndEffectorIntake;
import frc.robot.Commands.SwerveDrive;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.Climber;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.EndEffector;
import frc.robot.Subsystems.GroundIntake;
import frc.robot.Subsystems.Shintake;

public class RobotContainer {

  public static final Drivetrain m_drivetrain = Drivetrain.getInstance();

  public static final CommandXboxController driverController =
      new CommandXboxController(Constants.ControllerConstants.driverControllerPort);
  public static final CommandXboxController operatorController =
      new CommandXboxController(Constants.ControllerConstants.operatorControllerPort);

  public final JoystickButton resetHeading_Start =
      new JoystickButton(driverController.getHID(), XboxController.Button.kStart.value);

  public final SendableChooser<Command> autoChooser;

  public static final Shintake m_shintake = new Shintake();
  public static final EndEffector m_endEffector = new EndEffector();
  public static final Arm m_arm = new Arm();
  public static final GroundIntake m_groundIntake = new GroundIntake();
  public static final Climber m_climber = new Climber();

  public RobotContainer() {
    m_drivetrain.setDefaultCommand(new SwerveDrive());
    // Add all the choices of Autonomous modes to the Smart Dashboard
    autoChooser = AutoBuilder.buildAutoChooser();

    configureBindings();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private void configureBindings() {
    // Driver controller
    resetHeading_Start.onTrue(new InstantCommand(m_drivetrain::zeroHeading, m_drivetrain));

    // Operator controller
    operatorController
        .y()
        .whileTrue(
            new StartEndCommand(
                (() -> m_shintake.setRollersSpeed(1.0)),
                (() -> m_shintake.setRollersSpeed(0.0)),
                m_shintake)); // Y Shintake Shoot

    operatorController
        .b()
        .whileTrue(
            new StartEndCommand(
                (() -> m_shintake.setRollersSpeed(-0.4)),
                (() -> m_shintake.setRollersSpeed(0.0)),
                m_shintake)); // B Shintake Intake

    operatorController
        .x()
        .whileTrue(
            new EndEffectorIntake(
                m_endEffector,
                Constants.EndEffectorConstants.ControlConstants.pincherInSpeed)); // X Pincher out

    operatorController
        .a()
        .whileTrue(
            new EndEffectorIntake(
                m_endEffector,
                Constants.EndEffectorConstants.ControlConstants.pincherOutSpeed)); // A Pincher in

    operatorController
        .povUp()
        .whileTrue(
            new StartEndCommand(
                () -> m_arm.setArmSpeed(-0.3),
                () -> m_arm.setArmSpeed(0),
                m_arm)); // Up arrow arm rotates to front
    operatorController
        .povDown()
        .whileTrue(
            new StartEndCommand(
                () -> m_arm.setArmSpeed(0.3),
                () -> m_arm.setArmSpeed(0),
                m_arm)); // Down arrow arm rotates to back

    // left bumper eject intake
    operatorController
        .leftBumper()
        .whileTrue(
            new StartEndCommand(
                () -> m_groundIntake.setRollerSpeed(0.6),
                () -> m_groundIntake.setRollerSpeed(0),
                m_groundIntake));
    // right bumper intake in
    operatorController
        .rightBumper()
        .whileTrue(
            new StartEndCommand(
                () -> m_groundIntake.setRollerSpeed(-0.6),
                () -> m_groundIntake.setRollerSpeed(0),
                m_groundIntake));

    // left arrow pivot down
    operatorController
        .povLeft()
        .whileTrue(
            new StartEndCommand(
                () -> m_groundIntake.setPivotSpeed(-0.3),
                () -> m_groundIntake.setPivotSpeed(0),
                m_groundIntake));
    // right arrow pivot up
    operatorController
        .povRight()
        .whileTrue(
            new StartEndCommand(
                () -> m_groundIntake.setPivotSpeed(0.3),
                () -> m_groundIntake.setPivotSpeed(0),
                m_groundIntake));

    // left center button climber down
    operatorController
        .back()
        .whileTrue(
            new StartEndCommand(
                () -> m_climber.setMotorSpeed(-0.9),
                () -> m_climber.setMotorSpeed(0.0),
                m_climber));

    // right center button climb up
    operatorController
        .start()
        .whileTrue(
            new StartEndCommand(
                () -> m_climber.setMotorSpeed(0.9), () -> m_climber.setMotorSpeed(0.0), m_climber));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public void registerNamedCommands() {
    NamedCommands.registerCommand(
        "Reset Swerve Encoders",
        new InstantCommand(() -> m_drivetrain.resetAllEncoders())
            .withDeadline(new InstantCommand(() -> new WaitCommand(0.1))));

    NamedCommands.registerCommand(
        "Reset Heading",
        new InstantCommand(() -> m_drivetrain.zeroHeading())
            .withDeadline(new InstantCommand(() -> new WaitCommand(0.1))));

    NamedCommands.registerCommand(
        "Auton Reset",
        new InstantCommand(() -> m_drivetrain.autonReset())
            .withDeadline(new InstantCommand(() -> new WaitCommand(0.1))));
  }
}
