package frc.robot;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Commands.ArmController;
import frc.robot.Commands.EndEffectorController;
import frc.robot.Commands.GroundIntakeController;
import frc.robot.Commands.ShootSetSpeeds;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.EndEffector;
import frc.robot.Subsystems.GroundIntake;
import frc.robot.Subsystems.Shintake;

public class Macros {


    public static SequentialCommandGroup A1_DEALGAE_MACRO( Arm arm, Shintake shintake, EndEffector endEffector, GroundIntake groundIntake){
        SequentialCommandGroup command = new SequentialCommandGroup(
    
        //HOLDING A1 UNTIL ALGAE DETECTED
        new ParallelCommandGroup(
            new ArmController(arm, Constants.ArmConstants.ControlConstants.A1Position),
            new EndEffectorController(endEffector, Constants.EndEffectorConstants.ControlConstants.pincherInSpeed)
        )
        .until(()-> endEffector.getAlgaeDetected()),
        //ALGAE INTAKE RUN EXTENSION AFTER DETECTION
       new ParallelRaceGroup(
            new ParallelDeadlineGroup(new WaitCommand(0.1), new EndEffectorController(endEffector,  Constants.EndEffectorConstants.ControlConstants.pincherInSpeed)),
            new ArmController(arm, Constants.ArmConstants.ControlConstants.A1Position ) //TUNE TIMEOUT AS NEEDED
       ),
       //HANDOFF ANGLE AND GROUND INTAKE OUT PREP
       new ParallelCommandGroup(
            new ArmController(arm, Constants.ArmConstants.ControlConstants.handoffPosition),
            new GroundIntakeController(groundIntake, 0.19, 0.0), //TUNE ANGLE
            new ShootSetSpeeds(shintake, -0.3)
       )
       .until(() -> Math.abs(arm.getEncoderPosition()-Constants.ArmConstants.ControlConstants.handoffPosition) <0.01 ), //WHEN ARM IN ERROR, CONTINUE ON

       //HANDOFF AND BEGIN TRANSFER TO HOLD
        new ParallelCommandGroup(
            new ArmController(arm, Constants.ArmConstants.ControlConstants.handoffPosition),
            new SequentialCommandGroup(
                new WaitCommand(0.0), //TUNE
                new EndEffectorController(endEffector, Constants.EndEffectorConstants.ControlConstants.pincherInSpeed) //SMALL DELAY TO ENSURE FF INITIALIZES Constants.EndEffectorConstants.ControlConstants.pincherInSpeed
            ),
            new ShootSetSpeeds(shintake, -0.5),
            new GroundIntakeController(groundIntake, 0.19, 0.0)
        ).until(()->groundIntake.getAlgaeDetected()), //MOVES ON WHEN IN FIRST PART OF HOLD

        //CONTINUE TO MOVE BALL THROUGH TO INTAKE HOLD
        new ParallelCommandGroup(
            new SequentialCommandGroup( //SAFETY WAIT COMMAND
                new WaitCommand(0.3), //TUNE
                new ArmController(arm, Constants.ArmConstants.ControlConstants.storedPosition)
            ),
            new ShootSetSpeeds(shintake, -0.5).withTimeout(0.2), //TUNE
            new GroundIntakeController(groundIntake, 0.19, 0.3).withTimeout(0.3) //TUNE
        ),
        
        //SHOULD NOT REACH HERE, BUT HOLD POSITION IF ARMCONTROLLER FINISHES FOR SOME REASON
        new GroundIntakeController(groundIntake, 0.175, 0)
    );
        return command;
      }
    
      public static SequentialCommandGroup A2_DEALGAE_MACRO( Arm arm, Shintake shintake, EndEffector endEffector, GroundIntake groundIntake){
        SequentialCommandGroup command = new SequentialCommandGroup(
    
            //HOLDING A1 UNTIL ALGAE DETECTED
            new ParallelCommandGroup(
                new ArmController(arm, Constants.ArmConstants.ControlConstants.A2Position),
                new EndEffectorController(endEffector, Constants.EndEffectorConstants.ControlConstants.pincherInSpeed)
            )
            .until(()-> endEffector.getAlgaeDetected()),
            //ALGAE INTAKE RUN EXTENSION AFTER DETECTION
           new ParallelRaceGroup(
                new ParallelDeadlineGroup(new WaitCommand(0.1), new EndEffectorController(endEffector,  Constants.EndEffectorConstants.ControlConstants.pincherInSpeed)),
                new ArmController(arm, Constants.ArmConstants.ControlConstants.A2Position ) //TUNE TIMEOUT AS NEEDED
           ),
           //HANDOFF ANGLE AND GROUND INTAKE OUT PREP
           new ParallelCommandGroup(
                new ArmController(arm, Constants.ArmConstants.ControlConstants.handoffPosition),
                new GroundIntakeController(groundIntake, 0.19, 0.0), //TUNE ANGLE
                new ShootSetSpeeds(shintake, -0.3)
           )
           .until(() -> Math.abs(arm.getEncoderPosition()-Constants.ArmConstants.ControlConstants.handoffPosition) <0.01 ), //WHEN ARM IN ERROR, CONTINUE ON
    
           //HANDOFF AND BEGIN TRANSFER TO HOLD
            new ParallelCommandGroup(
                new ArmController(arm, Constants.ArmConstants.ControlConstants.handoffPosition),
                new SequentialCommandGroup(
                    new WaitCommand(0.0), //TUNE
                    new EndEffectorController(endEffector, Constants.EndEffectorConstants.ControlConstants.pincherInSpeed) //SMALL DELAY TO ENSURE FF INITIALIZES Constants.EndEffectorConstants.ControlConstants.pincherInSpeed
                ),
                new ShootSetSpeeds(shintake, -0.5),
                new GroundIntakeController(groundIntake, 0.19, 0.0)
            ).until(()->groundIntake.getAlgaeDetected()), //MOVES ON WHEN IN FIRST PART OF HOLD
    
            //CONTINUE TO MOVE BALL THROUGH TO INTAKE HOLD
            new ParallelCommandGroup(
                new SequentialCommandGroup( //SAFETY WAIT COMMAND
                    new WaitCommand(0.3), //TUNE
                    new ArmController(arm, Constants.ArmConstants.ControlConstants.storedPosition)
                ),
                new ShootSetSpeeds(shintake, -0.5).withTimeout(0.2), //TUNE
                new GroundIntakeController(groundIntake, 0.19, 0.3).withTimeout(0.3) //TUNE
            ),
            
            //SHOULD NOT REACH HERE, BUT HOLD POSITION IF ARMCONTROLLER FINISHES FOR SOME REASON
            new GroundIntakeController(groundIntake, 0.175, 0)
        );
        return command;
    }

  public static SequentialCommandGroup GROUND_INTAKE_DOWN(GroundIntake groundIntake) {
    SequentialCommandGroup command =
        new SequentialCommandGroup(
            new GroundIntakeController(groundIntake, 0.23, -1.0) // desired angle 0.175 speed: -1
                .until(() -> groundIntake.getAlgaeDetected()),
            new GroundIntakeController(groundIntake, 0.13, 0)
                );
    return command;
  }

  public static SequentialCommandGroup GROUND_INTAKE_UP(GroundIntake groundIntake) {
    SequentialCommandGroup command =
        new SequentialCommandGroup(
            new GroundIntakeController(groundIntake, 0.06, 0.0)
                );
          
    return command;
  }

  public static SequentialCommandGroup CLIMB_PREP(Arm arm, GroundIntake groundIntake) {
    SequentialCommandGroup command =
        new SequentialCommandGroup(
            new ParallelCommandGroup(
                new ArmController(arm, 0.715), new GroundIntakeController(groundIntake, 0.2, 0.0)));
    return command;
  }



  //   public static SequentialCommandGroup A1_DEALGAE_MACRO(
//       Arm arm, Shintake shintake, EndEffector endEffector, GroundIntake groundIntake) {

//     SequentialCommandGroup command =
//         new SequentialCommandGroup(
//             new SequentialCommandGroup(
//                 new ParallelCommandGroup(
//                         new ArmController(arm, Constants.ArmConstants.ControlConstants.A1Position),
//                         new EndEffectorController(
//                             endEffector,
//                             Constants.EndEffectorConstants.ControlConstants.pincherInSpeed),
//                         new GroundIntakeController(groundIntake, 0.175, 0.0))
//                     .until(() -> endEffector.getAlgaeDetected()),
//                 new ParallelCommandGroup(
//                         new SequentialCommandGroup(
//                                 new ArmController(
//                                         arm,
//                                         Constants.ArmConstants.ControlConstants.handoffPosition)
//                                     .until(
//                                         () ->
//                                             (Math.abs(
//                                                     arm.getEncoder().get()
//                                                         - Constants.ArmConstants.ControlConstants
//                                                             .handoffPosition)
//                                                 < 0.01)))
//                             .andThen(
//                                 new ParallelCommandGroup(
//                                     new ArmController(
//                                         arm,
//                                         Constants.ArmConstants.ControlConstants.handoffPosition),
//                                     new EndEffectorController(
//                                         endEffector,
//                                         Constants.EndEffectorConstants.ControlConstants
//                                             .pincherInSpeed),
//                                 new GroundIntakeController(groundIntake, 0.175, 0.0)),
//                         new StartEndCommand(
//                             () -> shintake.setRollersSpeed(0.4),
//                             () -> shintake.setRollersSpeed(0),
//                             shintake)))
//                     .until(() -> groundIntake.getAlgaeDetected()),
//                 new ParallelCommandGroup(
//                     (new StartEndCommand(
//                             () -> shintake.setRollersSpeed(0.45),
//                             () -> shintake.setRollersSpeed(0),
//                             shintake)
//                         .withTimeout(0.75)),
//                     new ArmController(
//                         arm, Constants.ArmConstants.ControlConstants.storedPosition))));

//     return command;
//   }

//   public static SequentialCommandGroup A2_DEALGAE_MACRO(
//       Arm arm, Shintake shintake, EndEffector endEffector, GroundIntake groundIntake) {
//     SequentialCommandGroup command =
//         new SequentialCommandGroup(
//             new SequentialCommandGroup(
//                 new ParallelCommandGroup(
//                         new ArmController(arm, Constants.ArmConstants.ControlConstants.A2Position),
//                         new EndEffectorController(
//                             endEffector,
//                             Constants.EndEffectorConstants.ControlConstants.pincherInSpeed))
//                     .until(() -> endEffector.getAlgaeDetected()),
//                 new ParallelCommandGroup(
//                         new SequentialCommandGroup(
//                                 new ArmController(
//                                         arm,
//                                         Constants.ArmConstants.ControlConstants.handoffPosition)
//                                     .until(
//                                         () ->
//                                             (Math.abs(
//                                                     arm.getEncoder().get()
//                                                         - Constants.ArmConstants.ControlConstants
//                                                             .handoffPosition)
//                                                 < 0.005)))
//                             .andThen(
//                                 new ParallelCommandGroup(
//                                     new ArmController(
//                                         arm,
//                                         Constants.ArmConstants.ControlConstants.handoffPosition),
//                                     new EndEffectorController(
//                                         endEffector,
//                                         Constants.EndEffectorConstants.ControlConstants
//                                             .pincherInSpeed),
//                                 new GroundIntakeController(groundIntake, 0.175, 0.0)),
//                         new StartEndCommand(
//                             () -> shintake.setRollersSpeed(-0.4),
//                             () -> shintake.setRollersSpeed(0),
//                             shintake)))
//                     .until(() -> groundIntake.getAlgaeDetected()),
//                 new ParallelCommandGroup(
//                     (new StartEndCommand(
//                             () -> shintake.setRollersSpeed(-0.25),
//                             () -> shintake.setRollersSpeed(0),
//                             shintake)
//                         .withTimeout(0.25)),
//                     new GroundIntakeController(groundIntake, 0.175, 0.3).withTimeout(0.535),
//                     new ArmController(
//                         arm, Constants.ArmConstants.ControlConstants.storedPosition))));

//     return command;
//   }
}
