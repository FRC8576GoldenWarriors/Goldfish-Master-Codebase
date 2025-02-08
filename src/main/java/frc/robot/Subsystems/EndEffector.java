// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drivers.WarriorSparkMax;
import frc.robot.Constants;

public class EndEffector extends SubsystemBase {

  private WarriorSparkMax pincherMotor;
  private DigitalInput intakeSensor;

  public EndEffector() {

    pincherMotor =
        new WarriorSparkMax(
            Constants.EndEffectorConstants.pincherID,
            MotorType.kBrushless,
            Constants.EndEffectorConstants.motorIsInverted,
            IdleMode.kBrake,
            50);

    intakeSensor = new DigitalInput(Constants.EndEffectorConstants.pincherDigiSensorID);
  }

  public void setSpeed(double speed) {
    // pincherMotor.set(speed);

    if (pincherMotor.getOutputCurrent() > 50) {
      pincherMotor.set(0);
    } else {
      pincherMotor.set(speed);
    }
  }

  public DigitalInput getDigitalInput() {
    return intakeSensor;
  }

  public boolean hasAlgae() {
    return intakeSensor.get();
  }

  @Override
  public void periodic() {}
}
