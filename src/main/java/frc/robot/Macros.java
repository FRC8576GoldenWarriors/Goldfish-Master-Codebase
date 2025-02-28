package frc.robot;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.Commands.ArmController;
import frc.robot.Commands.EndEffectorController;
import frc.robot.Commands.GroundIntakeController;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.EndEffector;
import frc.robot.Subsystems.GroundIntake;
import frc.robot.Subsystems.Shintake;

public class Macros {
  public static SequentialCommandGroup A1_DEALGAE_MACRO(
      Arm arm, Shintake shintake, EndEffector endEffector, GroundIntake groundIntake) {

    SequentialCommandGroup command =
        new SequentialCommandGroup(
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new ArmController(arm, Constants.ArmConstants.ControlConstants.A1Position),
                        new EndEffectorController(
                            endEffector,
                            Constants.EndEffectorConstants.ControlConstants.pincherInSpeed),
                        new GroundIntakeController(groundIntake, 0.175, 0.3))
                    .until(() -> endEffector.getAlgaeDetected()),
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new ArmController(
                                        arm,
                                        Constants.ArmConstants.ControlConstants.handoffPosition)
                                    .until(
                                        () ->
                                            (Math.abs(
                                                    arm.getEncoder().get()
                                                        - Constants.ArmConstants.ControlConstants
                                                            .handoffPosition)
                                                < 0.005)))
                            .andThen(
                                new ParallelCommandGroup(
                                    new ArmController(
                                        arm,
                                        Constants.ArmConstants.ControlConstants.handoffPosition),
                                    new EndEffectorController(
                                        endEffector,
                                        Constants.EndEffectorConstants.ControlConstants
                                            .pincherInSpeed)),
                                new GroundIntakeController(groundIntake, 0.175, 0.0)),
                        new StartEndCommand(
                            () -> shintake.setRollersSpeed(0.4),
                            () -> shintake.setRollersSpeed(0),
                            shintake))
                    .until(() -> groundIntake.getAlgaeDetected()),
                new ParallelCommandGroup(
                    (new StartEndCommand(
                            () -> shintake.setRollersSpeed(0.25),
                            () -> shintake.setRollersSpeed(0),
                            shintake)
                        .withTimeout(0.25) // .
                    // andThen(new StartEndCommand( //replace with instant or runnable command later
                    //   () -> shintake.setRollersSpeed(0.75, 0.8), //tune
                    //   () -> shintake.setRollersSpeed(0.75, 0.8),
                    //   shintake))
                    ),
                    new ArmController(
                        arm, Constants.ArmConstants.ControlConstants.storedPosition))));

    return command;
  }

  public static SequentialCommandGroup A2_DEALGAE_MACRO(
      Arm arm, Shintake shintake, EndEffector endEffector, GroundIntake groundIntake) {
    SequentialCommandGroup command =
        new SequentialCommandGroup(
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new ArmController(arm, Constants.ArmConstants.ControlConstants.A2Position),
                        new EndEffectorController(
                            endEffector,
                            Constants.EndEffectorConstants.ControlConstants.pincherInSpeed))
                    .until(() -> endEffector.getAlgaeDetected()),
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new ArmController(
                                        arm,
                                        Constants.ArmConstants.ControlConstants.handoffPosition)
                                    .until(
                                        () ->
                                            (Math.abs(
                                                    arm.getEncoder().get()
                                                        - Constants.ArmConstants.ControlConstants
                                                            .handoffPosition)
                                                < 0.005)))
                            .andThen(
                                new ParallelCommandGroup(
                                    new ArmController(
                                        arm,
                                        Constants.ArmConstants.ControlConstants.handoffPosition),
                                    new EndEffectorController(
                                        endEffector,
                                        Constants.EndEffectorConstants.ControlConstants
                                            .pincherInSpeed)),
                                new GroundIntakeController(groundIntake, 0.175, 0.0)),
                        new StartEndCommand(
                            () -> shintake.setRollersSpeed(0.4),
                            () -> shintake.setRollersSpeed(0),
                            shintake))
                    .until(() -> groundIntake.getAlgaeDetected()),
                new ParallelCommandGroup(
                    (new StartEndCommand(
                            () -> shintake.setRollersSpeed(0.25),
                            () -> shintake.setRollersSpeed(0),
                            shintake)
                        .withTimeout(0.25)
                        .andThen(
                            new StartEndCommand( // replace with instant or runnable command later
                                () -> shintake.setRollersSpeed(0.75, 0.8), // tune
                                () -> shintake.setRollersSpeed(0.75, 0.8),
                                shintake))),
                    new GroundIntakeController(groundIntake, 0.175, 0.3).withTimeout(0.535),
                    new ArmController(
                        arm, Constants.ArmConstants.ControlConstants.storedPosition))));

    return command;
  }

  public static SequentialCommandGroup GROUND_INTAKE_DOWN(GroundIntake groundIntake) {
    SequentialCommandGroup command =
        new SequentialCommandGroup(
            new GroundIntakeController(groundIntake, 0.175, 0.3)
                .until(() -> groundIntake.getAlgaeDetected()));
    return command;
  }

  public static SequentialCommandGroup GROUND_INTAKE_UP(GroundIntake groundIntake) {
    SequentialCommandGroup command =
        new SequentialCommandGroup(
            new GroundIntakeController(groundIntake, 0.09, 0.0)
                .until(() -> groundIntake.getAlgaeDetected()));
    return command;
  }

  public static SequentialCommandGroup CLIMB_PREP(Arm arm, GroundIntake groundIntake){
    SequentialCommandGroup command = new SequentialCommandGroup(new ParallelCommandGroup(new ArmController(arm, 0.715), new GroundIntakeController(groundIntake, 0.2, 0.0)));
    return command;
  }
}
