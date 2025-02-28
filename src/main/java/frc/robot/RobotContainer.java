package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Commands.AlignToAprilTag;
import frc.robot.Commands.ArmController;
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
  public static AprilTagStatsLimelight bargeTagStatsLimelight;

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
      bargeTagStatsLimelight =
          new AprilTagStatsLimelight(
              Constants.VisionConstants.LimelightConstants.BargeLimelightConstants
                  .BARGE_NETWORKTABLE_KEY);

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

    if (SimConstants.currentMode.equals(SimConstants.Mode.REAL)) {
      resetHeading_Start.onTrue(new InstantCommand(m_drivetrain::zeroHeading, m_drivetrain));

      // Driver controller

      driverController
          .leftBumper()
          .whileTrue(new AlignToAprilTag(bargeTagStatsLimelight, m_drivetrain));

      driverController
          .rightBumper()
          .whileTrue(
              new ParallelCommandGroup(
                  new StartEndCommand(
                      () -> m_groundIntake.setRollerSpeed(-0.3),
                      () -> m_groundIntake.setRollerSpeed(0),
                      m_groundIntake),
                  new StartEndCommand(
                      () -> m_shintake.setRollersSpeed(0.65, 0.70),
                      () -> m_shintake.setRollersSpeed(0),
                      m_shintake)));

    //   driverController
    //       .start()
    //       .whileTrue(
    //           new StartEndCommand(
    //               () -> m_climber.setMotorSpeed(0.9),
    //               () -> m_climber.setMotorSpeed(0.0),
    //               m_climber));

    //   driverController
    //       .back()
    //       .whileTrue(
    //           new StartEndCommand(
    //               () -> m_climber.setMotorSpeed(-0.9),
    //               () -> m_climber.setMotorSpeed(0.0),
    //               m_climber));

        driverController.povDown().whileTrue(new StartEndCommand(()-> m_endEffector.setSpeed(0.2), ()->m_endEffector.setSpeed(0.0), m_endEffector));
        driverController.povUp().whileTrue(new ArmController(m_arm, 0.715));
      // operator controller

      operatorController
          .b()
          .onTrue(Macros.A1_DEALGAE_MACRO(m_arm, m_shintake, m_endEffector, m_groundIntake));

      operatorController
          .y()
          .onTrue(Macros.A2_DEALGAE_MACRO(m_arm, m_shintake, m_endEffector, m_groundIntake));

      operatorController.a().onTrue(Macros.GROUND_INTAKE_DOWN(m_groundIntake));

      operatorController.x().onTrue(Macros.GROUND_INTAKE_UP(m_groundIntake));

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
    operatorController
    .povRight()
    .onTrue(
        new GroundIntakeController(m_groundIntake, 0.2, 0));


        
      // right bumper intake in

      // right arrow pivot up
      operatorController.povRight().onTrue(new GroundIntakeController(m_groundIntake, 0.07, 0.0));

      // left center button climber down
      operatorController.povLeft().onTrue(new GroundIntakeController(m_groundIntake, 0.17, 0.0));

      
    }
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public void registerNamedCommands() {
    if (SimConstants.currentMode.equals(SimConstants.Mode.REAL)) {
      //   NamedCommands.registerCommand(
      //       "Reset Swerve Encoders",
      //       new InstantCommand(() -> m_drivetrain.resetAllEncoders())
      //           .withDeadline(new InstantCommand(() -> new WaitCommand(0.1))));

      //   NamedCommands.registerCommand(
      //       "Reset Heading",
      //       new InstantCommand(() -> m_drivetrain.zeroHeading())
      //           .withDeadline(new InstantCommand(() -> new WaitCommand(0.1))));

      NamedCommands.registerCommand(
          "Reset Climb",
          new InstantCommand(() -> m_climber.setMotorSpeed(0.2))
              .withDeadline(new InstantCommand(() -> new WaitCommand(0.3))));

      NamedCommands.registerCommand(
          "Shooter RP",
          new InstantCommand(() -> m_endEffector.setSpeed(1))
              .withDeadline(new InstantCommand(() -> new WaitCommand(4))));
      //   NamedCommands.registerCommand(
      //       "Auton Reset",
      //       new InstantCommand(() -> m_drivetrain.autonReset())
      //           .withDeadline(new InstantCommand(() -> new WaitCommand(0.1))));
    }
  }
}
