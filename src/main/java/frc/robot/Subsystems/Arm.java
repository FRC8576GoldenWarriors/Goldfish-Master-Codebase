// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.lib.drivers.WarriorSparkMax;
import frc.robot.Constants;

import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volt;

import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
  /** Creates a new AlgaeArm. */
  private WarriorSparkMax armMotor;
  private SysIdRoutine routine;
  private DutyCycleEncoder armAbsEncoder;

  public Arm() {
    armMotor =
        new WarriorSparkMax(
            Constants.ArmConstants.HardwareConstants.armMotorID,
            MotorType.kBrushless,
            Constants.ArmConstants.HardwareConstants.motorIsInverted,
            IdleMode.kBrake,
            45);

    armAbsEncoder = new DutyCycleEncoder(Constants.ArmConstants.HardwareConstants.armEncoderDIO);
    routine = new SysIdRoutine(new SysIdRoutine.Config(), new Mechanism(armMotor::setVoltage, log -> {
                // Record a frame for the shooter motor.
                log.motor("shooter-wheel")
                    .voltage(
                        getVoltage())
                    .angularPosition(getAngle())
                    .angularVelocity(getAngularVelocity()
                        );
              },this));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    Logger.recordOutput("Arm/Arm_Voltage", armMotor.getBusVoltage());
    Logger.recordOutput("Arm/Arm_Current", armMotor.getOutputCurrent());
    Logger.recordOutput("Arm/Arm_Encoder_Value", getEncoderPosition());
    Logger.recordOutput("Arm/Arm_Velocity", getArmVelocity());
  }

  public void setArmVoltage(double volts) {
    armMotor.setVoltage(volts);
  }

  public void setArmSpeed(double speed) {
    armMotor.set(speed);
  }

  public void readLogs(){
    
    Logger.recordOutput("Arm/Arm_Voltage", armMotor.getBusVoltage());
    Logger.recordOutput("Arm/Arm_Current", armMotor.getOutputCurrent());
    Logger.recordOutput("Arm/Arm_Encoder_Value", getEncoderPosition());
    Logger.recordOutput("Arm/Arm_Velocity", getArmVelocity());
  }
  public SparkMax getMotor() {
    return armMotor;
  }
  public Angle getAngle(){
    return Angle.ofBaseUnits(getArmVelocity(), Rotation);
  }

  // radians per second
  public double getArmVelocity() {
    return (getMotor().getEncoder().getVelocity() / 60.0); // rpm/60
  }

  public DutyCycleEncoder getDutyCycleEncoder() {
    return armAbsEncoder;
  }

  public AngularVelocity getAngularVelocity(){
    return AngularVelocity.ofBaseUnits(getArmVelocity(), RotationsPerSecond);
  }
  public double getEncoderPosition() {
    return armAbsEncoder.get();
  }

  public DutyCycleEncoder getEncoder() {
    return armAbsEncoder;
  }
   
  public Voltage getVoltage(){
    return Voltage.ofBaseUnits(armMotor.getBusVoltage(),Volt);
  }
  public Command sysidQualitistic(SysIdRoutine.Direction direction){
    return routine.quasistatic(direction);
  }
  public Command sysidDynamic(SysIdRoutine.Direction direction){
    return routine.dynamic(direction);
  }
}
