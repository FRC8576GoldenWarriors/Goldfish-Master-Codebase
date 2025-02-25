package frc.robot;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.Commands.ArmController;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.EndEffector;
import frc.robot.Subsystems.Shintake;

public class Macros {
  public static SequentialCommandGroup GET_A1_DEALGAE_MACRO(
      Arm arm, Shintake shintake, EndEffector endEffector) {
    SequentialCommandGroup command =
        new SequentialCommandGroup(
            new SequentialCommandGroup(
                    new ParallelCommandGroup(
                        new ArmController(arm, Constants.ArmConstants.ControlConstants.A1Position),
                        new StartEndCommand(
                            () ->
                                endEffector.setSpeed(
                                    Constants.EndEffectorConstants.ControlConstants.pincherInSpeed),
                            () -> endEffector.setSpeed(0),
                            endEffector)))
                .until(() -> endEffector.getAlgaeDetected()));

    return command;
  }
}
