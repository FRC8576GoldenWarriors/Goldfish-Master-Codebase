package frc.robot.Subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType; 
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode; 
import com.revrobotics.RelativeEncoder; 
import frc.lib.drivers.WarriorSparkMax;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbMech extends SubsystemBase {

    private final WarriorSparkMax climbMotor;
    private final RelativeEncoder climbEncoder;

    public ClimbMech() {
        
        climbMotor = new WarriorSparkMax(
            Constants.ClimberConstants.motorID,
            MotorType.kBrushless, 
            Constants.ClimberConstants.motorIsInverted,
            IdleMode.kBrake 
        );

        
        climbEncoder = climbMotor.getEncoder();
    }

    
    public void windRope() {
        climbMotor.set(0.5); // Adjust speed as needed
    }

    
    public void unwindRope() {
        climbMotor.set(-0.5); // Adjust speed as needed
    }

    
    public void stop() {
        climbMotor.set(0);
    }

    
    public void setMotorVoltage(double voltage) {
        climbMotor.setVoltage(voltage);
    }

    
    public double getEncoderPosition() {
        return climbEncoder.getPosition(); 
    }

    
    public double getEncoderVelocity() {
        return climbEncoder.getVelocity(); 
    }

    // Reset the encoder to zero
    public void resetEncoder() {
        climbEncoder.setPosition(0);
    }

    @Override
    public void periodic() {
        // Optionally log encoder values to SmartDashboard or other telemetry
    }
}
 
