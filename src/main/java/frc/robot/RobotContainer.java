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
import frc.robot.Commands.AlignToAprilTag;
import frc.robot.Commands.ArmController;
import frc.robot.Commands.EndEffectorController;
import frc.robot.Commands.GroundIntakeController;
import frc.robot.Commands.SimSwerveDrive;
import frc.robot.Commands.SwerveDrive;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.Climber;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.EndEffector;
import frc.robot.Subsystems.GroundIntake;
import frc.robot.Subsystems.LEDStrip;
import frc.robot.Subsystems.Limelight.AprilTagStatsLimelight;
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
  public static LEDStrip m_led;

  public static AprilTagStatsLimelight reefTagStatsLimelight;
  // public static AprilTagStatsLimelight bargeTagStatsLimelight;

  public static DrivetrainSim m_drivetrainSim;
  public static SimEndEffector m_SimEndEffector;

  public RobotContainer() {

    if (SimConstants.currentMode.equals(SimConstants.Mode.REAL)) {
      // System.out.println("is real");
      m_drivetrain = Drivetrain.getInstance();
      m_shintake = new Shintake();
      m_endEffector = new EndEffector();
      m_arm = new Arm();
      m_groundIntake = new GroundIntake();
      m_climber = new Climber();
      m_led = new LEDStrip(1, 25);

      reefTagStatsLimelight =
          new AprilTagStatsLimelight(
              Constants.VisionConstants.LimelightConstants.ReefLimelightConstants
                  .REEF_NETWORKTABLE_KEY);
      //      bargeTagStatsLimelight =
      //          new AprilTagStatsLimelight(
      //              Constants.VisionConstants.LimelightConstants.BargeLimelightConstants
      //                  .BARGE_NETWORKTABLE_KEY);

      m_drivetrain.setDefaultCommand(new SwerveDrive());
    } else if (SimConstants.currentMode.equals(SimConstants.Mode.SIM)) {
      // System.out.println("is sim");
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
    // Driver controller

    if (SimConstants.currentMode.equals(SimConstants.Mode.REAL)) {
      resetHeading_Start.onTrue(new InstantCommand(m_drivetrain::zeroHeading, m_drivetrain));

      // Operator controller
      //   operatorController
      //       .y()
      //       .whileTrue(
      //           new StartEndCommand(
      //               (() -> m_shintake.setRollersSpeed(0.85, 0.80)), // 1.0 0.95
      //               (() -> m_shintake.setRollersSpeed(0.0)),
      //               m_shintake)); // Y Shintake Shoot

      operatorController
          .b()
          .onTrue(
              new SequentialCommandGroup(
                  new SequentialCommandGroup(
                      new ParallelCommandGroup(
                              new ArmController(
                                  m_arm, Constants.ArmConstants.ControlConstants.A1Position),
                              new EndEffectorController(
                                  m_endEffector,
                                  Constants.EndEffectorConstants.ControlConstants.pincherInSpeed))
                          .until(() -> m_endEffector.getAlgaeDetected()),
                      new ParallelCommandGroup(
                              new SequentialCommandGroup(
                                      new ArmController(
                                              m_arm,
                                              Constants.ArmConstants.ControlConstants
                                                  .handoffPosition)
                                          .until(
                                              () ->
                                                  (Math.abs(
                                                          m_arm.getEncoder().get()
                                                              - Constants.ArmConstants
                                                                  .ControlConstants.handoffPosition)
                                                      < 0.005)))
                                  .andThen(
                                      new ParallelCommandGroup(
                                          new ArmController(
                                              m_arm,
                                              Constants.ArmConstants.ControlConstants
                                                  .handoffPosition),
                                          new EndEffectorController(
                                              m_endEffector,
                                              Constants.EndEffectorConstants.ControlConstants
                                                  .pincherInSpeed)), 
                                                  new GroundIntakeController(m_groundIntake, 0.175, 0.0)), 
                              new StartEndCommand(
                                  () -> m_shintake.setRollersSpeed(0.4),
                                  () -> m_shintake.setRollersSpeed(0),
                                  m_shintake))
                          .until(() -> m_groundIntake.getAlgaeDetected()),
                      new ParallelCommandGroup(
                          (new StartEndCommand(
                                  () -> m_shintake.setRollersSpeed(0.25),
                                  () -> m_shintake.setRollersSpeed(0),
                                  m_shintake)
                              .withTimeout(0.25).
                              andThen(new StartEndCommand( //replace with instant or runnable command later
                                () -> m_shintake.setRollersSpeed(0.75, 0.8), //tune
                                () -> m_shintake.setRollersSpeed(0.75, 0.8),
                                m_shintake))),
                              new GroundIntakeController(m_groundIntake, 0.175, 0.3).withTimeout(0.535), 
                              
                          new ArmController(
                              m_arm, Constants.ArmConstants.ControlConstants.storedPosition))

                               
                              
                              )));


    operatorController
    .rightBumper()
    .whileTrue(new ParallelCommandGroup( new StartEndCommand(
    () -> m_groundIntake.setRollerSpeed(-0.3),
    () -> m_groundIntake.setRollerSpeed(0),
    m_groundIntake), new StartEndCommand(()->m_shintake.setRollersSpeed(0.75,0.8), ()->m_shintake.setRollersSpeed(0), m_shintake)));

      operatorController.y().onTrue(new ArmController(m_arm, 0.72));

      operatorController
          .x()
          .whileTrue(
              new EndEffectorController(
                  m_endEffector,
                  Constants.EndEffectorConstants.ControlConstants.pincherInSpeed)); // X Pincher out

      operatorController
          .a()
          .whileTrue(
              new EndEffectorController(
                  m_endEffector,
                  Constants.EndEffectorConstants.ControlConstants.pincherOutSpeed)); // A Pincher in

      operatorController
          .povUp()
          .whileTrue(
              new StartEndCommand(
                  () -> m_groundIntake.setPivotSpeed(-0.3),
                  () -> m_groundIntake.setPivotSpeed(0),
                  m_groundIntake)); // Up arrow arm rotates to front
      operatorController
          .povDown()
          .whileTrue(
              new StartEndCommand(
                  () -> m_groundIntake.setPivotSpeed(0.3),
                  () -> m_groundIntake.setPivotSpeed(0),
                  m_groundIntake)); // Down arrow arm rotates to back

      // left bumper eject intake
      operatorController
          .leftBumper()
          .whileTrue(
              new StartEndCommand(
                  () -> m_groundIntake.setRollerSpeed(0.3),
                  () -> m_groundIntake.setRollerSpeed(0),
                  m_groundIntake));
      // right bumper intake in


     
      // right arrow pivot up
      operatorController
          .povRight()
          .onTrue(
              new GroundIntakeController(m_groundIntake, 0.07, 0.0));

      // left center button climber down
      operatorController
          .povLeft()
          .onTrue(new GroundIntakeController(m_groundIntake, 0.17, 0.0));

      // right center button climb up
      operatorController
          .start()
          .whileTrue(
              new StartEndCommand(
                  () -> m_climber.setMotorSpeed(0.9),
                  () -> m_climber.setMotorSpeed(0.0),
                  m_climber));

      // up arrow align reef
      // driverController.povUp().whileTrue(new AlignToAprilTag(reefTagStatsLimelight,
      // m_drivetrain));

      // down arrow align barge
      driverController
          .rightBumper()
          .whileTrue(new AlignToAprilTag(reefTagStatsLimelight, m_drivetrain));
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

      //   NamedCommands.registerCommand(
      //       "Auton Reset",
      //       new InstantCommand(() -> m_drivetrain.autonReset())
      //           .withDeadline(new InstantCommand(() -> new WaitCommand(0.1))));
    }
  }
}
