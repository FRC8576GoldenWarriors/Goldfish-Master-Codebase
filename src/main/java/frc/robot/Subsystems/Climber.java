package frc.robot.Subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drivers.WarriorSparkMax;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {

  private final WarriorSparkMax climbMotor;
  private final DutyCycleEncoder climbEncoder;
  private boolean isClimbing;

  public Climber() {

    climbMotor =
        new WarriorSparkMax(
            Constants.ClimberConstants.HardwareConstants.motorID,
            MotorType.kBrushless,
            Constants.ClimberConstants.HardwareConstants.motorIsInverted,
            IdleMode.kBrake);

    climbEncoder =
        new DutyCycleEncoder(
            Constants.ClimberConstants.HardwareConstants.climberEncoderDIO,
            1.0,
            Constants.ClimberConstants.ControlConstants.climberEncoderOffset);
    climbEncoder.setInverted(Constants.ClimberConstants.HardwareConstants.climberEncoderIsInverted);
  }

  public void windRope(double speed) {
    climbMotor.set(speed); // Adjust speed as needed
  }

  public void stop() {
    climbMotor.set(0);
  }

  public void setClimbing(boolean isClimbing) {
    this.isClimbing = isClimbing;
  }

  public boolean isClimbing() {
    return this.isClimbing;
  }

  public boolean isClimbingUp() {
    return isClimbing;
  }

  public void setMotorVoltage(double voltage) {
    climbMotor.setVoltage(voltage);
  }

  public void setMotorSpeed(double speed) {
    climbMotor.set(speed);
  }

  public double getEncoderPosition() {
    return climbEncoder.get();
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Climber/Climb_Voltage", climbMotor.getBusVoltage());
    Logger.recordOutput("Climber/Climb_Current", climbMotor.getOutputCurrent());
    Logger.recordOutput("Climber/Climb_Position", getEncoderPosition());
    Logger.recordOutput("Climber/Climbing", isClimbingUp());
  }
}
