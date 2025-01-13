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
    /**
     * Creates a new CANSparkMax with the necessary configurations.
     * @param deviceId The device ID.
     * @param m The motor type (Brushed/Brushless).
     * @param mode The idle mode (kBrake/kCoast).
     * @param limit The current limit.
     * @param isInverted The invert type of the motor.
     */
    private SparkMaxConfig config;
    public WarriorSparkMax(int deviceId, MotorType motorType, boolean inverted, IdleMode brakeMode){//IdleMode mode, int limit, boolean isInverted){
        super(deviceId, motorType);
        config = new SparkMaxConfig();
        config
        .inverted(inverted)
        .idleMode(brakeMode);
        
        this.configure(config,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
        //this.restoreFactoryDefaults();
        //this.setSmartCurrentLimit(limit);
        // this.setInverted(isInverted);
        // this.setIdleMode(mode);
        //this.burnFlash();
        String key = "Spark " + this.getDeviceId() + " Flashes";
        Preferences.setDouble(key, Preferences.getDouble(key, 0) + 1);
    }
    public WarriorSparkMax(int deviceId, MotorType motorType, boolean inverted, IdleMode brakeMode, int currentLimit){//IdleMode mode, int limit, boolean isInverted){
        super(deviceId, motorType);
        config = new SparkMaxConfig();
        config
        .inverted(inverted)
        .idleMode(brakeMode)
        .smartCurrentLimit(currentLimit);
        
        this.configure(config,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
        //this.restoreFactoryDefaults();
        //this.setSmartCurrentLimit(limit);
        // this.setInverted(isInverted);
        // this.setIdleMode(mode);
        //this.burnFlash();
        String key = "Spark " + this.getDeviceId() + " Flashes";
        Preferences.setDouble(key, Preferences.getDouble(key, 0) + 1);
    }






    public void setInverted(boolean isInverted){
        config.inverted(isInverted);
        this.configure(config,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
    }

    public void setIdleMode(IdleMode brakeMode){
        config.idleMode(brakeMode);
        this.configure(config,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
    }

    public void setCurrentLimit(int currentLimit){
        config.smartCurrentLimit(currentLimit);
        this.configure(config,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
    }
}