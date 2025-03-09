// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import java.util.Map;

public class LEDStrip extends SubsystemBase {
  private int port;
  private int length;

  private AddressableLED led;
  private AddressableLEDBuffer buffer;

  public LEDStrip(int port, int length) {
    this.port = port;
    this.length = length;

    led = new AddressableLED(port);
    buffer = new AddressableLEDBuffer(length);

    led.setLength(length);
    led.start();
  }

  public void setPattern(LEDPattern pattern) {
    pattern.applyTo(buffer);
  }

  public void rainbowScroll() {
    SmartDashboard.putString(getName(), "rainbowScroll");

    Map<Double, Color> maskSteps =
        Map.of(
            0.0,
            Color.kWhite,
            Constants.LEDConstants.PatternConfig.kLEDRainbowScrollSize,
            Color.kBlack);
    LEDPattern base = LEDPattern.rainbow(255, 255);
    LEDPattern mask =
        LEDPattern.steps(maskSteps)
            .scrollAtRelativeSpeed(
                Percent.per(Second)
                    .of(Constants.LEDConstants.PatternConfig.kLEDRainbowScrollSpeed));

    LEDPattern pattern = base.mask(mask);

    setPattern(pattern);
  }

  public void climbProgress() {
    double percent =
        RobotContainer.m_climber.getEncoderPosition()
            / Constants.ClimberConstants.ControlConstants.climberUpPosition;
    LEDPattern progress = LEDPattern.progressMaskLayer(() -> percent);
    LEDPattern base = Constants.LEDConstants.PatternConfig.kLEDNoStatusBreathe;
    LEDPattern pattern = base.mask(progress);

    setPattern(pattern);
  }

  public void scroll(LEDPattern pattern, double speed) {
    setPattern(pattern.scrollAtRelativeSpeed(Percent.per(Second).of(speed)));
  }

  public void blink(LEDPattern pattern, double speed) {
    SmartDashboard.putString(getName(), "blink");
    setPattern(pattern.blink(Seconds.of(speed)));
  }

  public void breathe(LEDPattern pattern, double speed) {
    SmartDashboard.putString(getName(), "breathe");

    setPattern(pattern.breathe(Second.of(speed)).scrollAtRelativeSpeed(Percent.per(Second).of(25)));
  }

  public void progress(LEDPattern mask, double percentage) {
    SmartDashboard.putString(getName(), "progress");

    LEDPattern base = LEDPattern.progressMaskLayer(() -> percentage);
    setPattern(base.mask(mask));
  }

  public void solid(LEDPattern pattern) {
    SmartDashboard.putString(getName(), "Solid");
    setPattern(pattern);
  }

  @Override
  public void periodic() {
    // testing inputs change later

    if (DriverStation.isDisabled()) { // disabled
      scroll(
          Constants.LEDConstants.PatternConfig.kLEDDisabledScroll,
          Constants.LEDConstants.PatternConfig.kLEDDisabledScrollSpeed);
    } else if (RobotContainer.m_climber.isClimbing()) {
      climbProgress();
    } else if (RobotContainer.bargeTagStatsLimelight.isTagReached()) {
      solid(Constants.LEDConstants.PatternConfig.kShooterIsReady);

    } else if (RobotContainer.bargeTagStatsLimelight.isTagDetected()) {
      breathe(
          Constants.LEDConstants.PatternConfig.kAprilTags,
          Constants.LEDConstants.PatternConfig.kAprilTagBlinkSpeed);
    }

    // else if (driverController.getLeftBumper()) {
    //   blink(
    //       Constants.LEDConstants.PatternConfig.kAprilTags,
    //       Constants.LEDConstants.PatternConfig.kAprilTagBlinkSpeed);

    // }
    else if (RobotContainer.m_groundIntake.getAlgaeDetected()) { // algae ground intake/hold
      breathe(
          Constants.LEDConstants.PatternConfig.kLEDAlgaeGroundBreathe,
          Constants.LEDConstants.PatternConfig.kLEDAlgaeGroundBreatheSpeed);

    } else if (RobotContainer.m_endEffector.getAlgaeDetected()) { // algae end effector
      blink(
          Constants.LEDConstants.PatternConfig.kLEDAlgaePincherBlink,
          Constants.LEDConstants.PatternConfig.kLEDAlgaePincherBlinkSpeed);
    }

    // } else if (!RobotContainer.m_endEffector.getCoralDigitalInput().get()) { // coral detected
    //   breathe(
    //       Constants.LEDConstants.PatternConfig.kLEDCoralDetectedBreathe,
    //       Constants.LEDConstants.PatternConfig.kLEDCoralDetectedBreatheSpeed);
    // } else if (RobotContainer.m_endEffector.motorRunning()) { //?
    //   blink(
    //       Constants.LEDConstants.PatternConfig.kLEDCoralIntakeBlink,
    //       Constants.LEDConstants.PatternConfig.kLEDCoralIntakeBlinkSpeed); //supposed to be
    // coral motor running
    // } else if (RobotContainer.m_endEffector.motorRunning()) { //?
    //   blink(
    //       Constants.LEDConstants.PatternConfig.kLEDCoralIntakeBlink,
    //       Constants.LEDConstants.PatternConfig.kLEDCoralIntakeBlinkSpeed); //supposed to be
    // coral motor running
    //  }
    else { // gold breathe idle
      scroll(Constants.LEDConstants.PatternConfig.kLEDNoStatusBreathe, 50);
    }

    led.setData(buffer);
  }
}
