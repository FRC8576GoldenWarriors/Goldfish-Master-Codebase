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
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Commands.AlignToAprilTag;
import frc.robot.Commands.ArmController;
import frc.robot.Commands.EndEffectorController;
import frc.robot.Commands.GroundIntakeController;
import frc.robot.Commands.SimSwerveDrive;
import frc.robot.Commands.SwerveDrive;
import frc.robot.Commands.SwitchCamera;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.Camera;
import frc.robot.Subsystems.Climber;
import frc.robot.Subsystems.DriverStream;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.EndEffector;
import frc.robot.Subsystems.GroundIntake;
import frc.robot.Subsystems.LEDStrip;
import frc.robot.Subsystems.Limelight.AprilTagStatsLimelight;
import frc.robot.Subsystems.Shintake;
import frc.robot.Subsystems.Simulation.DrivetrainSim;
import frc.robot.Subsystems.Simulation.SimConstants;
import frc.robot.Subsystems.Simulation.SimEndEffector;

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
  public static Camera m_DriverCamera;
  public static Camera m_CageCamera;
  public static DriverStream m_CameraStream;

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

      m_DriverCamera =
          new Camera(Constants.VisionConstants.CameraConstants.DRIVER_CAMERA_NAME, 320, 240, true);
      m_CageCamera =
          new Camera(Constants.VisionConstants.CameraConstants.CAGE_CAMERA_NAME, 320, 240, true);
      m_CameraStream = new DriverStream(m_DriverCamera, m_CageCamera);

      reefTagStatsLimelight =
          new AprilTagStatsLimelight(
              Constants.VisionConstants.LimelightConstants.ReefLimelightConstants
                  .REEF_NETWORKTABLE_KEY);
      bargeTagStatsLimelight =
          new AprilTagStatsLimelight(
              Constants.VisionConstants.LimelightConstants.BargeLimelightConstants
                  .BARGE_NETWORKTABLE_KEY);

      m_drivetrain.setDefaultCommand(new SwerveDrive());
      registerNamedCommands();
    } else if (SimConstants.currentMode.equals(SimConstants.Mode.SIM)) {
      // System.out.println("is sim");
      m_drivetrainSim = DrivetrainSim.getInstance();
      m_SimEndEffector = new SimEndEffector();
      m_drivetrainSim.setDefaultCommand(new SimSwerveDrive());
    } else {
      //   String logPath = LogFileUtil.findReplayLog();
      //   Logger.setReplaySource(new WPILOGReader(logPath));
      //   Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
    }
    // Add all the choices of Autonomous modes to the Smart Dashboar
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
          .or(driverController.leftBumper().and(driverController.rightBumper())) //or left and right bumper to allow double binding
          .whileTrue(new AlignToAprilTag(bargeTagStatsLimelight, m_drivetrain));

      driverController
          .rightBumper() 
          .or(driverController.leftBumper().and(driverController.rightBumper())) //or left and right bumper to allow double binding
          .whileTrue(
              new ParallelCommandGroup(
                  new StartEndCommand(
                      () -> m_groundIntake.setRollerSpeed(-0.45), // -0.3
                      () -> m_groundIntake.setRollerSpeed(0),
                      m_groundIntake),
                  new StartEndCommand(
                      () -> m_shintake.setRollersSpeed(0.83571, 0.9), // 0.9286, 1.0 // 0.65 0.7
                      () -> m_shintake.setRollersSpeed(0),
                      m_shintake)));

      driverController
          .y()
          .whileTrue(
              new StartEndCommand(
                  () -> m_climber.setMotorSpeed(0.9),
                  () -> m_climber.setMotorSpeed(0.0),
                  m_climber));

      driverController
          .b()
          .whileTrue(
              new StartEndCommand(
                  () -> m_climber.setMotorSpeed(-0.9),
                  () -> m_climber.setMotorSpeed(0.0),
                  m_climber));

      // operator controller

      operatorController
          .y()
          .onTrue(Macros.A2_DEALGAE_MACRO(m_arm, m_shintake, m_endEffector, m_groundIntake));

      operatorController
          .b()
          .onTrue(Macros.A1_DEALGAE_MACRO(m_arm, m_shintake, m_endEffector, m_groundIntake));

      operatorController.x().onTrue(Macros.GROUND_INTAKE_UP(m_groundIntake));

      operatorController.a().onTrue(Macros.GROUND_INTAKE_DOWN(m_groundIntake));

      operatorController
          .start()
          .whileTrue(
              new StartEndCommand(
                  () ->
                      m_endEffector.setSpeed(
                          Constants.EndEffectorConstants.ControlConstants.pincherInSpeed),
                  () -> m_endEffector.setSpeed(0.0),
                  m_endEffector));
      operatorController.povLeft().onTrue(new GroundIntakeController(m_groundIntake, 0.09, 0.0));

      operatorController.povRight().onTrue(new GroundIntakeController(m_groundIntake, 0.23, -0.8));

      // processor
      operatorController
          .rightBumper()
          .whileTrue(
              new ParallelCommandGroup(
                  new StartEndCommand(
                      () -> m_shintake.setRollersSpeed(-0.4, 0),
                      () -> m_shintake.setRollersSpeed(0),
                      m_shintake),
                  new StartEndCommand(
                      () -> m_groundIntake.setRollerSpeed(1.0),
                      () -> m_groundIntake.setRollerSpeed(0),
                      m_groundIntake)));
      operatorController
          .leftBumper()
          .whileTrue(
              new StartEndCommand(
                  () -> m_groundIntake.setRollerSpeed(-0.4),
                  () -> m_groundIntake.setRollerSpeed(0),
                  m_groundIntake));
    }
    driverController.back().onTrue(new SwitchCamera(m_CameraStream));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public void registerNamedCommands() {

    //   NamedCommands.registerCommand(
    //       "Reset Swerve Encoders",
    //       new InstantCommand(() -> m_drivetrain.resetAllEncoders())
    //           .withDeadline(new InstantCommand(() -> new WaitCommand(0.1))));

    //   NamedCommands.registerCommand(
    //       "Reset Heading",
    //       new InstantCommand(() -> m_drivetrain.zeroHeading())
    //           .withDeadline(new InstantCommand(() -> new WaitCommand(0.1))));

    NamedCommands.registerCommand(
        "Reset Climb", new InstantCommand(() -> m_climber.setMotorSpeed(0.9)).withTimeout(3.0));

    NamedCommands.registerCommand(
        "Shooter RP",
        new StartEndCommand(
                () -> m_endEffector.setSpeed(-1.0), () -> m_endEffector.setSpeed(0), m_endEffector)
            .withTimeout(
                5)); // InstantCommand(() -> m_endEffector.setSpeed(-1.0)).withTimeout(4.0));
    NamedCommands.registerCommand(
        "Dealgae Part 1 A1",
        new ParallelCommandGroup(
                new ArmController(m_arm, Constants.ArmConstants.ControlConstants.A1Position),
                new EndEffectorController(
                    m_endEffector, Constants.EndEffectorConstants.ControlConstants.pincherInSpeed),
                new GroundIntakeController(m_groundIntake, 0.175, 0.3))
            .until(() -> m_endEffector.getAlgaeDetected()));

    NamedCommands.registerCommand(
        "Dealgae Part 1 A2",
        new ParallelCommandGroup(
                new ArmController(m_arm, Constants.ArmConstants.ControlConstants.A2Position),
                new EndEffectorController(
                    m_endEffector, Constants.EndEffectorConstants.ControlConstants.pincherInSpeed),
                new GroundIntakeController(m_groundIntake, 0.175, 0.3))
            .until(() -> m_endEffector.getAlgaeDetected()));

    NamedCommands.registerCommand(
        "Dealgae Part 2",
        (new SequentialCommandGroup(
            new ParallelCommandGroup(
                    new SequentialCommandGroup(
                            new ArmController(
                                    m_arm, Constants.ArmConstants.ControlConstants.handoffPosition)
                                .until(
                                    () ->
                                        (Math.abs(
                                                m_arm.getEncoder().get()
                                                    - Constants.ArmConstants.ControlConstants
                                                        .handoffPosition)
                                            < 0.005)))
                        .andThen(
                            new ParallelCommandGroup(
                                new ArmController(
                                    m_arm, Constants.ArmConstants.ControlConstants.handoffPosition),
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
                    .withTimeout(0.25)),
                new GroundIntakeController(m_groundIntake, 0.175, 0.3).withTimeout(0.535),
                new ArmController(
                    m_arm, Constants.ArmConstants.ControlConstants.storedPosition)))));
    // .withTimeout(5.0)); // tune timeouts

    NamedCommands.registerCommand(
        "Shoot",
        new ParallelCommandGroup(
                new StartEndCommand(
                        () -> m_groundIntake.setRollerSpeed(-0.45), // -0.3
                        () -> m_groundIntake.setRollerSpeed(0),
                        m_groundIntake)
                    .withTimeout(2),
                new StartEndCommand(
                    () -> m_shintake.setRollersSpeed(0.83571, 0.9), // 0.9286, 1.0 // 0.65 0.7
                    () -> m_shintake.setRollersSpeed(0),
                    m_shintake))
            .withTimeout(2));
  }
}
