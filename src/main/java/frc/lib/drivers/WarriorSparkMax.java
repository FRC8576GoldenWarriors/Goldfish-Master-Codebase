// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.drivers;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.Preferences;

/** Add your docs here. */
public class WarriorSparkMax extends SparkMax {

  private SparkMaxConfig config;

  public WarriorSparkMax(int deviceId, MotorType motorType, boolean inverted, IdleMode brakeMode) {
    super(deviceId, motorType);
    config = new SparkMaxConfig();
    config.inverted(inverted).idleMode(brakeMode);

    this.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    String key = "Spark " + this.getDeviceId() + " Flashes";
    Preferences.setDouble(key, Preferences.getDouble(key, 0) + 1);
  }

  public WarriorSparkMax(
      int deviceId, MotorType motorType, boolean inverted, IdleMode brakeMode, int currentLimit) {
    super(deviceId, motorType);
    config = new SparkMaxConfig();
    config.inverted(inverted).idleMode(brakeMode).smartCurrentLimit(currentLimit);

    this.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    String key = "Spark " + this.getDeviceId() + " Flashes";
    Preferences.setDouble(key, Preferences.getDouble(key, 0) + 1);
  }

  public void setInverted(boolean isInverted) {
    config.inverted(isInverted);
    this.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setIdleMode(IdleMode brakeMode) {
    config.idleMode(brakeMode);
    this.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setCurrentLimit(int currentLimit) {
    config.smartCurrentLimit(currentLimit);
    this.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setkP(double kP) {
    config.closedLoop.p(kP);
    this.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setkI(double kI) {
    config.closedLoop.i(kI);
    this.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setkD(double kD) {
    config.closedLoop.d(kD);
    this.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setkF(double ff) {
    config.closedLoop.velocityFF(ff);
    this.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setMaxMotion(double maxVel, double maxAccel) {
    config.closedLoop.maxMotion.maxVelocity(maxVel).maxAcceleration(maxAccel);
    this.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setOutputRange(double minOutput, double maxOutput) {
    config.closedLoop.outputRange(minOutput, maxOutput);
    this.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }
}
