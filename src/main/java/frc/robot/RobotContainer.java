package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Commands.AlignToAprilTag;
import frc.robot.Commands.ArmController;
import frc.robot.Commands.ClimberController;
import frc.robot.Commands.EndEffectorController;
import frc.robot.Commands.GroundIntakeController;
import frc.robot.Commands.ShootRPM;
import frc.robot.Commands.ShootSetSpeeds;
import frc.robot.Commands.SimSwerveDrive;
import frc.robot.Commands.SwerveDrive;
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

      //   m_DriverCamera =
      //       new Camera(Constants.VisionConstants.CameraConstants.DRIVER_CAMERA_NAME, 320, 240,
      // 30, true);
      //   m_CageCamera =
      //       new Camera(Constants.VisionConstants.CameraConstants.CAGE_CAMERA_NAME, 320, 240, 30,
      // true);
      //   m_CameraStream = new DriverStream(m_DriverCamera, m_CageCamera);

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
          .or(
              driverController
                  .leftBumper()
                  .and(
                      driverController
                          .rightBumper())) // or left and right bumper to allow double binding
          .whileTrue(new AlignToAprilTag(bargeTagStatsLimelight, m_drivetrain));

        driverController
            .rightBumper()
            .or(
                driverController
                    .leftBumper()
                    .and(
                        driverController
                            .rightBumper())) // or left and right bumper to allow double binding
            .whileTrue(
                new ParallelCommandGroup(
                    new StartEndCommand(
                        () -> m_groundIntake.setRollerSpeed(-0.45), // -0.3
                        () -> m_groundIntake.setRollerSpeed(0),
                        m_groundIntake),
                    new StartEndCommand(
                        () -> m_shintake.setRollersSpeed(0.743, 0.75), // 0.9286, 1.0 // 0.65
                        () -> m_shintake.setRollersSpeed(0),
                        m_shintake)));

      driverController
          .rightBumper()
          .whileTrue(
              new ParallelCommandGroup(
                  new ShootRPM(m_shintake, 4850, 5300),
                  new GroundIntakeController(m_groundIntake, 0.13, -0.45)));
        
       

            

      driverController.y().onTrue(
        new ClimberController(m_climber, Constants.ClimberConstants.ControlConstants.climberUpPosition)
        );
   
      driverController.b().onTrue(
        new SequentialCommandGroup(
            new ClimberController(m_climber, 0.02).until(()->Math.abs(m_climber.getEncoderPosition()-0.01)<0.005),
            new StartEndCommand(()-> m_climber.setMotorSpeed(Constants.ClimberConstants.ControlConstants.climbDownSpeed), ()-> m_climber.setMotorSpeed(0), m_climber)
            .withTimeout(2.8)
            )
        );
    
        driverController
        .x()
        .whileTrue(
            new StartEndCommand(
                () -> m_climber.setMotorSpeed(0.9),
                () -> m_climber.setMotorSpeed(0.0),
                m_climber));

    driverController
        .a()
        .whileTrue(
            new StartEndCommand(
                () -> m_climber.setMotorSpeed(-0.9),
                () -> m_climber.setMotorSpeed(0.0),
                m_climber));

      

        

      // driverController.back().onTrue(new SwitchCamera(m_CameraStream));


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
   
                //unslack
        operatorController.povUp().onTrue(new ClimberController(m_climber, 0.02));
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
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public void registerNamedCommands() {

   
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
                    m_endEffector, Constants.EndEffectorConstants.ControlConstants.pincherInSpeed))
            .until(() -> m_endEffector.getAlgaeDetected()));

    NamedCommands.registerCommand(
        "Dealgae Part 1 A2",
        new ParallelCommandGroup(
                new ArmController(m_arm, Constants.ArmConstants.ControlConstants.A2Position),
                new EndEffectorController(
                    m_endEffector, Constants.EndEffectorConstants.ControlConstants.pincherInSpeed))
            .until(() -> m_endEffector.getAlgaeDetected()));

    NamedCommands.registerCommand(
        "Dealgae Part 2",
        new SequentialCommandGroup(
            new ParallelRaceGroup(
            new ParallelDeadlineGroup(new WaitCommand(0.1), new EndEffectorController(m_endEffector,  Constants.EndEffectorConstants.ControlConstants.pincherInSpeed)),
            new ArmController(m_arm, Constants.ArmConstants.ControlConstants.A2Position ) //TUNE TIMEOUT AS NEEDED
       ),
       //HANDOFF ANGLE AND GROUND INTAKE OUT PREP
       new ParallelCommandGroup(
            new ArmController(m_arm, Constants.ArmConstants.ControlConstants.handoffPosition),
            new GroundIntakeController(m_groundIntake, 0.19, 0.0), //TUNE ANGLE
            new ShootSetSpeeds(m_shintake, -0.3)
       )
       .until(() -> Math.abs(m_arm.getEncoderPosition()-Constants.ArmConstants.ControlConstants.handoffPosition) <0.01 ), //WHEN ARM IN ERROR, CONTINUE ON

       //HANDOFF AND BEGIN TRANSFER TO HOLD
        new ParallelCommandGroup(
            new ArmController(m_arm, Constants.ArmConstants.ControlConstants.handoffPosition),
            new SequentialCommandGroup(
                new WaitCommand(0.0), //TUNE
                new EndEffectorController(m_endEffector, Constants.EndEffectorConstants.ControlConstants.pincherInSpeed) //SMALL DELAY TO ENSURE FF INITIALIZES Constants.EndEffectorConstants.ControlConstants.pincherInSpeed
            ),
            new ShootSetSpeeds(m_shintake, -0.5),
            new GroundIntakeController(m_groundIntake, 0.19, 0.0)
        ).until(()->m_groundIntake.getAlgaeDetected()), //MOVES ON WHEN IN FIRST PART OF HOLD

        //CONTINUE TO MOVE BALL THROUGH TO INTAKE HOLD
        new ParallelCommandGroup(
            new SequentialCommandGroup( //SAFETY WAIT COMMAND
                new WaitCommand(0.3), //TUNE
                new ArmController(m_arm, Constants.ArmConstants.ControlConstants.storedPosition)
            ),
            new ShootSetSpeeds(m_shintake, -0.3).withTimeout(0.35), //TUNE
            new GroundIntakeController(m_groundIntake, 0.19, 0.3).withTimeout(0.4) //TUNE
        ),
        
        //SHOULD NOT REACH HERE, BUT HOLD POSITION IF ARMCONTROLLER FINISHES FOR SOME REASON
        new GroundIntakeController(m_groundIntake, 0.18, 0)
        )   
        .withTimeout(5) //TUNE LATER
    );
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
            .withTimeout(1.4));

    NamedCommands.registerCommand(
        "Reset Climber",
        new SequentialCommandGroup(
            new ClimberController(m_climber, 0.02).until(()->Math.abs(m_climber.getEncoderPosition()-0.02)<0.005),
            new StartEndCommand(()-> m_climber.setMotorSpeed(Constants.ClimberConstants.ControlConstants.climbDownSpeed), ()-> m_climber.setMotorSpeed(0), m_climber)
            .withTimeout(2.8)
        )
    );
  }

  
}
