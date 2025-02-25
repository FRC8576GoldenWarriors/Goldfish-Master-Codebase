package frc.robot;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Commands.ArmController;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.Shintake;

public class Macros {
  public static SequentialCommandGroup dealgaeL1(Arm arm, Shintake shintake) {
    return new SequentialCommandGroup(
        new ParallelCommandGroup(
            new ArmController(arm, 0.25),
            new InstantCommand(() -> shintake.setRollersSpeed(-0.3, -0.3))));
  }
}
