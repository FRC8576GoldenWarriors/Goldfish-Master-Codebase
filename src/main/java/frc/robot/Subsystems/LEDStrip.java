// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;

import java.util.Map;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class LEDStrip extends SubsystemBase {
  private int port = 0;
  private int length = 36;

  private AddressableLED led;
  private AddressableLEDBuffer buffer;
  
  public LEDStrip() {
    led = new AddressableLED(port);
    buffer = new AddressableLEDBuffer(length);
    
    led.setLength(length);
    led.start();
  }

  public void setPattern(LEDPattern pattern) {
    pattern.applyTo(buffer);
  }

  public void doScroll() {
    Map<Double, Color> maskSteps = Map.of(0.0, Color.kWhite, 0.5, Color.kBlack);
    LEDPattern base = LEDPattern.rainbow(255, 255);
    LEDPattern mask =
      LEDPattern.steps(maskSteps).scrollAtRelativeSpeed(Percent.per(Second).of(200));

    LEDPattern pattern = base.mask(mask);

    setPattern(pattern);
  }

  @Override
  public void periodic() {
    // scrolling gradient rainbow
    // final LEDPattern m_rainbow = LEDPattern.rainbow(255, 128);
    // final Distance kLedSpacing = Meters.of(1 / 120.0);
    // final LEDPattern m_scrollingRainbow = m_rainbow.scrollAtAbsoluteSpeed(MetersPerSecond.of(1), kLedSpacing);
    // setPattern(m_scrollingRainbow);

    
    if (RobotContainer.driverController.getHID().getAButton()) {
      doScroll();
      SmartDashboard.putBoolean(getName(), true);
    } else {
      SmartDashboard.putBoolean(getName(), false);
      setPattern(LEDPattern.solid(Color.kRed));
    }

    led.setData(buffer);
  }
}
