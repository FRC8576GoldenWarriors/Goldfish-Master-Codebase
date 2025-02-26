package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Commands.ArmController;
import frc.robot.Commands.EndEffectorIntake;
import frc.robot.Commands.SimSwerveDrive;
import frc.robot.Commands.SwerveDrive;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.Climber;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.EndEffector;
import frc.robot.Subsystems.GroundIntake;
import frc.robot.Subsystems.LEDStrip;
import frc.robot.Subsystems.Shintake;
import frc.robot.Subsystems.Simulation.DrivetrainSim;
import frc.robot.Subsystems.Simulation.SimConstants;
import frc.robot.Subsystems.Simulation.SimEndEffector;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class RobotContainer {

  //   public static final Drivetrain m_drivetrain = Drivetrain.getInstance();

  public static final CommandXboxController driverController =
      new CommandXboxController(Constants.ControllerConstants.driverControllerPort);
  public static final CommandXboxController operatorController =
      new CommandXboxController(Constants.ControllerConstants.operatorControllerPort);

  public final JoystickButton resetHeading_Start =
      new JoystickButton(driverController.getHID(), XboxController.Button.kStart.value);

  public final SendableChooser<Command> autoChooser;

  public static Drivetrain m_drivetrain;
  public static Shintake m_shintake;
  public static EndEffector m_endEffector;
  public static Arm m_arm;
  public static GroundIntake m_groundIntake;
  public static Climber m_climber;
  public static LEDStrip m_ledStrip;

  public static DrivetrainSim m_drivetrainSim;
  public static SimEndEffector m_SimEndEffector;

  public RobotContainer() {

    if (SimConstants.currentMode.equals(SimConstants.Mode.REAL)) {
      System.out.println("is real");
      m_drivetrain = Drivetrain.getInstance();
      m_shintake = new Shintake();
      m_endEffector = new EndEffector();
      m_arm = new Arm();
      m_groundIntake = new GroundIntake();
      m_climber = new Climber();
      m_ledStrip =
          new LEDStrip(
              Constants.LEDConstants.HardwareConstants.kLEDPort,
              Constants.LEDConstants.HardwareConstants.kLEDLength);

      m_drivetrain.setDefaultCommand(new SwerveDrive());

    } else if (SimConstants.currentMode.equals(SimConstants.Mode.SIM)) {
      System.out.println("is sim");
      m_drivetrainSim = DrivetrainSim.getInstance();
      m_SimEndEffector = new SimEndEffector();
      m_drivetrainSim.setDefaultCommand(new SimSwerveDrive());
    } else {
      String logPath = LogFileUtil.findReplayLog();
      Logger.setReplaySource(new WPILOGReader(logPath));
      Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
    }
    // Add all the choices of Autonomous modes to the Smart Dashboard
    autoChooser = AutoBuilder.buildAutoChooser();

    configureBindings();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private void configureBindings() {

    if (SimConstants.currentMode.equals(SimConstants.Mode.REAL)) {
      resetHeading_Start.onTrue(new InstantCommand(m_drivetrain::zeroHeading, m_drivetrain));
      // Driver controller
      // driverController.y().onTrue(new ArmController(m_arm, 0.40));
      // driverController.b().onTrue(new ArmController(m_arm, 0.31));

      // Operator controller
      // operatorController.b().onTrue(Macros.GET_A1_DEALGAE_MACRO(m_arm, m_shintake,
      // m_endEffector));

      operatorController
          .b()
          .onTrue(
              new SequentialCommandGroup(
                  new SequentialCommandGroup(
                      new ParallelCommandGroup(
                              new ArmController(
                                  m_arm, Constants.ArmConstants.ControlConstants.A1Position),
                              new StartEndCommand(
                                  () ->
                                      m_endEffector.setSpeed(
                                          Constants.EndEffectorConstants.ControlConstants
                                              .pincherInSpeed),
                                  () -> m_endEffector.setSpeed(0),
                                  m_endEffector))
                          .until(() -> m_endEffector.getAlgaeDetected())),
                  new ParallelCommandGroup(
                      new SequentialCommandGroup(
                          new ArmController(
                              m_arm, Constants.ArmConstants.ControlConstants.transportPosition),
                          new StartEndCommand(
                              () -> m_shintake.setRollersSpeed(0.4),
                              () -> m_shintake.setRollersSpeed(0),
                              m_shintake)))));

      operatorController.y().onTrue(new ArmController(m_arm, 0.31));

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

      //   operatorController
      //       .povUp()
      //       .whileTrue(
      //           new StartEndCommand(
      //               () -> m_arm.setArmSpeed(0.3),
      //               () -> m_arm.setArmSpeed(0),
      //               m_arm)); // Up arrow arm rotates to front
      //   operatorController
      //       .povDown()
      //       .whileTrue(
      //           new StartEndCommand(
      //               () -> m_arm.setArmSpeed(-0.3),
      //               () -> m_arm.setArmSpeed(0),
      //               m_arm)); // Down arrow arm rotates to back

      //   // left bumper eject intake
      //   operatorController
      //       .leftBumper()
      //       .whileTrue(
      //           new StartEndCommand(
      //               () -> m_groundIntake.setRollerSpeed(0.3),
      //               () -> m_groundIntake.setRollerSpeed(0),
      //               m_groundIntake));
      //   // right bumper intake in

      //   // left arrow pivot down
      //   operatorController.povLeft().onTrue(new GroundIntakeController(m_groundIntake, 0.21,
      // -0.3));
      //   // right arrow pivot up
      //   operatorController.povRight().onTrue(new GroundIntakeController(m_groundIntake, 0.09,
      // 0));

      //   // left center button climber down
      //   operatorController
      //       .back()
      //       .whileTrue(
      //           new StartEndCommand(
      //               () -> m_climber.setMotorSpeed(-0.9),
      //               () -> m_climber.setMotorSpeed(0.0),
      //               m_climber));

      //   // right center button climb up
      //   operatorController
      //       .start()
      //       .whileTrue(
      //           new StartEndCommand(
      //               () -> m_climber.setMotorSpeed(0.9),
      //               () -> m_climber.setMotorSpeed(0.0),
      //               m_climber));
    }
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public void registerNamedCommands() {
    if (SimConstants.currentMode.equals(SimConstants.Mode.REAL)) {
      NamedCommands.registerCommand(
          "Reset Swerve Encoders",
          new InstantCommand(() -> m_drivetrain.resetAllEncoders())
              .withDeadline(new InstantCommand(() -> new WaitCommand(0.1))));

      NamedCommands.registerCommand(
          "Reset Heading",
          new InstantCommand(() -> m_drivetrain.zeroHeading())
              .withDeadline(new InstantCommand(() -> new WaitCommand(0.1))));
      NamedCommands.registerCommand(
          "Reset Shintake",
          new InstantCommand(() -> m_shintake.setRollersSpeed(0))
              .withDeadline(new InstantCommand(() -> new WaitCommand(0.1))));

      //   NamedCommands.registerCommand(
      //       "Auton Reset",
      //       new InstantCommand(() -> m_drivetrain.autonReset())
      //           .withDeadline(new InstantCommand(() -> new WaitCommand(0.1))));
    }
  }
}
