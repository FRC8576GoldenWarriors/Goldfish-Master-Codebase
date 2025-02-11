package frc.robot.Subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drivers.WarriorSparkMax;
import frc.robot.Constants;

public class Climber extends SubsystemBase {

  private final WarriorSparkMax climbMotor;
  private final RelativeEncoder climbEncoder;
  private boolean isClimbing;

  public Climber() {

    climbMotor =
        new WarriorSparkMax(
            Constants.ClimberConstants.HardwareConstants.motorID,
            MotorType.kBrushless,
            Constants.ClimberConstants.HardwareConstants.motorIsInverted,
            IdleMode.kBrake);

    climbEncoder = climbMotor.getEncoder();
  }

  public void windRope(double speed) {
    climbMotor.set(speed); // Adjust speed as needed
  }

  public void stop() {
    climbMotor.set(0);
  }

  public void climbing(boolean value) {
    isClimbing = value;
  }

  public boolean getClimbing() {
    return isClimbing;
  }

  public void setMotorVoltage(double voltage) {
    climbMotor.setVoltage(voltage);
  }

  public void setMotorSpeed(double speed) {
    climbMotor.set(speed);
  }

  public double getEncoderPosition() {
    return climbEncoder.getPosition();
  }

  public double getEncoderVelocity() {
    return climbEncoder.getVelocity();
  }

  public void resetEncoder() {
    climbEncoder.setPosition(0);
  }

  @Override
  public void periodic() {
    // Optionally log encoder values to SmartDashboard
  }
}
