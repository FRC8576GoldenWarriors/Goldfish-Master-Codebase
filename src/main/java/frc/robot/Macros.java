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
            new EndEffectorController(endEffector, Constants.EndEffectorConstants.ControlConstants.pincherAlgaeSpeed)
        )
        .until(()-> endEffector.getAlgaeDetected()),
        //ALGAE INTAKE RUN EXTENSION AFTER DETECTION
       new ParallelRaceGroup(
            new ParallelDeadlineGroup(new WaitCommand(0.1), new EndEffectorController(endEffector,  Constants.EndEffectorConstants.ControlConstants.pincherAlgaeSpeed)),
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
                new EndEffectorController(endEffector, Constants.EndEffectorConstants.ControlConstants.pincherAlgaeSpeed) //SMALL DELAY TO ENSURE FF INITIALIZES Constants.EndEffectorConstants.ControlConstants.pincherInSpeed
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
            new ShootSetSpeeds(shintake, -0.3).withTimeout(0.3), //TUNE
            new GroundIntakeController(groundIntake, 0.19, 0.3).withTimeout(0.4) //TUNE
        ),
        
        //SHOULD NOT REACH HERE, BUT HOLD POSITION IF ARMCONTROLLER FINISHES FOR SOME REASON
        new GroundIntakeController(groundIntake, 0.18, 0)
    );
        return command;
      }
    
      public static SequentialCommandGroup A2_DEALGAE_MACRO( Arm arm, Shintake shintake, EndEffector endEffector, GroundIntake groundIntake){
        SequentialCommandGroup command = new SequentialCommandGroup(
    
        //HOLDING A1 UNTIL ALGAE DETECTED
        new ParallelCommandGroup(
            new ArmController(arm, Constants.ArmConstants.ControlConstants.A2Position),
            new EndEffectorController(endEffector, Constants.EndEffectorConstants.ControlConstants.pincherAlgaeSpeed)
        )
        .until(()-> endEffector.getAlgaeDetected()),
        //ALGAE INTAKE RUN EXTENSION AFTER DETECTION
       new ParallelRaceGroup(
            new ParallelDeadlineGroup(new WaitCommand(0.1), new EndEffectorController(endEffector,  Constants.EndEffectorConstants.ControlConstants.pincherAlgaeSpeed)),
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
                new EndEffectorController(endEffector, Constants.EndEffectorConstants.ControlConstants.pincherAlgaeSpeed) //SMALL DELAY TO ENSURE FF INITIALIZES Constants.EndEffectorConstants.ControlConstants.pincherInSpeed
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
            new ShootSetSpeeds(shintake, -0.3).withTimeout(0.35), //TUNE
            new GroundIntakeController(groundIntake, 0.19, 0.3).withTimeout(0.4) //TUNE
        ),
        
        //SHOULD NOT REACH HERE, BUT HOLD POSITION IF ARMCONTROLLER FINISHES FOR SOME REASON
        new GroundIntakeController(groundIntake, 0.18, 0)
    );
        return command;
    }

    public static SequentialCommandGroup LOLIPOP_DEALGAE_MACRO( Arm arm, Shintake shintake, EndEffector endEffector, GroundIntake groundIntake){
        SequentialCommandGroup command = new SequentialCommandGroup(
    
        //HOLDING A1 UNTIL ALGAE DETECTED
        new ParallelCommandGroup(
            new ArmController(arm, Constants.ArmConstants.ControlConstants.LolipopPosition),
            new EndEffectorController(endEffector, Constants.EndEffectorConstants.ControlConstants.pincherAlgaeSpeed)
        )
        .until(()-> endEffector.getAlgaeDetected()),
        //ALGAE INTAKE RUN EXTENSION AFTER DETECTION
       new ParallelRaceGroup(
            new ParallelDeadlineGroup(new WaitCommand(0.1), new EndEffectorController(endEffector,  Constants.EndEffectorConstants.ControlConstants.pincherAlgaeSpeed)),
            new ArmController(arm, Constants.ArmConstants.ControlConstants.LolipopPosition) //TUNE TIMEOUT AS NEEDED
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
                new EndEffectorController(endEffector, Constants.EndEffectorConstants.ControlConstants.pincherAlgaeSpeed) //SMALL DELAY TO ENSURE FF INITIALIZES Constants.EndEffectorConstants.ControlConstants.pincherInSpeed
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
            new ShootSetSpeeds(shintake, -0.3).withTimeout(0.3), //TUNE
            new GroundIntakeController(groundIntake, 0.19, 0.3).withTimeout(0.4) //TUNE
        ),
        
        //SHOULD NOT REACH HERE, BUT HOLD POSITION IF ARMCONTROLLER FINISHES FOR SOME REASON
        new GroundIntakeController(groundIntake, 0.18, 0)
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
            new GroundIntakeController(groundIntake, 0.01, 0.0)
                );
          
    return command;
  }




  public static SequentialCommandGroup CORAL_L1(EndEffector endEffector, Arm arm){
    SequentialCommandGroup command = new SequentialCommandGroup(
        new ParallelCommandGroup(
            new ArmController(arm, Constants.ArmConstants.ControlConstants.coralStationPosition),
            new EndEffectorController(endEffector, Constants.EndEffectorConstants.ControlConstants.pincherCoralInSpeed)
        ).until(()->endEffector.getCoralDetected()),

        new ArmController(arm, Constants.ArmConstants.ControlConstants.L1Position)

    );

    return command;
  }

  public static SequentialCommandGroup CORAL_L2(EndEffector endEffector, Arm arm){
    SequentialCommandGroup command = new SequentialCommandGroup(
        new ParallelCommandGroup(
            new ArmController(arm, Constants.ArmConstants.ControlConstants.coralStationPosition),
            new EndEffectorController(endEffector, Constants.EndEffectorConstants.ControlConstants.pincherCoralInSpeed)
        ).until(()->endEffector.getCoralDetected()),

        new ArmController(arm, Constants.ArmConstants.ControlConstants.L2Position)
    );

    return command;
  }

  

}
