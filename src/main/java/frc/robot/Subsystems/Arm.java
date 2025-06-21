// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.lib.drivers.WarriorSparkMax;
import frc.robot.Constants;

import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
  /** Creates a new AlgaeArm. */
  private WarriorSparkMax armMotor;

  private DutyCycleEncoder armAbsEncoder;

  public static boolean isCoraling;

  public SysIdRoutine routine = new SysIdRoutine(new SysIdRoutine.Config(
  null,null,null,
  (state)->Logger.recordOutput("SysIdState",state.toString())
), new SysIdRoutine.Mechanism((voltage)->this.setArmVoltage(voltage.in(Volts)), null, this));

  public Arm() {
    armMotor =
        new WarriorSparkMax(
            Constants.ArmConstants.HardwareConstants.armMotorID,
            MotorType.kBrushless,
            Constants.ArmConstants.ControlConstants.motorIsInverted,
            IdleMode.kCoast,
            40);

    armAbsEncoder =
        new DutyCycleEncoder(
            Constants.ArmConstants.HardwareConstants.armEncoderDIO,
            1.0,
            Constants.ArmConstants.ControlConstants.armEncoderOffset);

    armAbsEncoder.setInverted(Constants.ArmConstants.ControlConstants.armEncoderIsInverted);

    isCoraling = false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    Logger.recordOutput("Arm/Arm_Voltage", armMotor.getAppliedOutput());
    Logger.recordOutput("Arm/Arm_Current", armMotor.getOutputCurrent());
    Logger.recordOutput("Arm/Arm_Encoder_Value", getEncoderPosition());
    Logger.recordOutput("Arm/Arm_Velocity", getArmVelocity());

    SmartDashboard.putNumber("Arm Enocder Position", getEncoderPosition());
    SmartDashboard.putNumber("Arm Motor Voltage", armMotor.getBusVoltage());
    SmartDashboard.putNumber("Arm Encoder Velocity", getArmVelocity());
  }

  public void setArmVoltage(double volts) {
    armMotor.setVoltage(volts);
  }

  public void setArmSpeed(double speed) {
    armMotor.set(speed);
  }

  public SparkMax getMotor() {
    return armMotor;
  }

  // radians per second
  public double getArmVelocity() {
    return (getMotor().getEncoder().getVelocity()); // rpm/60
  }

  public DutyCycleEncoder getDutyCycleEncoder() {
    return armAbsEncoder;
  }

  public double getEncoderPosition() {
    return armAbsEncoder.get();
  }

  public DutyCycleEncoder getEncoder() {
    return armAbsEncoder;
  }

  public void setArmMotorIdleMode(IdleMode idleMode) {
    armMotor.setIdleMode(idleMode);
  }

  public void setCoraling(boolean finished) {
    isCoraling = finished;
  }

  public boolean getCoraling() {
    return isCoraling;
  }
  
  public Command QualID(SysIdRoutine.Direction direction){
    switch(direction){
      case kForward:
        return routine.quasistatic(Direction.kForward).until(()->getEncoderPosition()>.4);
      case kReverse:
        return routine.quasistatic(Direction.kReverse).until(()->getEncoderPosition()<0.2);
    }
    return null;
  }

  public Command DynamicID(SysIdRoutine.Direction direction){
    switch(direction){
      case kForward:
        return routine.dynamic(Direction.kForward).until(()->getEncoderPosition()>.4);
      case kReverse:
        return routine.dynamic(Direction.kReverse).until(()->getEncoderPosition()<0.2);
    }
    return null;
  }
}
