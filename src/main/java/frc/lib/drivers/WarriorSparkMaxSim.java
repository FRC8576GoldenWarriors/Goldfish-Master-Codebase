// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.drivers;

import com.revrobotics.sim.SparkMaxSim;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;

/** Add your docs here. */
public class WarriorSparkMaxSim extends SparkMaxSim {

  double updateInterval;

  public WarriorSparkMaxSim(WarriorSparkMax motorToSim, DCMotor gearBox) {
    super(motorToSim, gearBox);
    this.updateInterval = 0.02; // default 20ms interval
  }

  public WarriorSparkMaxSim(WarriorSparkMax motorToSim, DCMotor gearBox, double updateInterval) {
    super(motorToSim, gearBox);
    this.updateInterval = updateInterval;
  }

  public void iterate() {
    this.iterate(this.getVelocity(), RoboRioSim.getVInVoltage(), this.updateInterval);
    updateVolatge();
  }

  public void iterate(double velocity) {
    this.iterate(velocity, RoboRioSim.getVInCurrent(), this.updateInterval);
    updateVolatge();
  }

  public void updateVolatge() {
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(this.getMotorCurrent()));
  }

  public double getInterval() {
    return updateInterval;
  }

  public void setInterval(double updateInterval) {
    this.updateInterval = updateInterval;
  }

  public void setAngle(double angle) {
    this.getAbsoluteEncoderSim().setPosition(angle);
  }

  public double getAngle() {
    return this.getAbsoluteEncoderSim().getPosition();
  }
}
