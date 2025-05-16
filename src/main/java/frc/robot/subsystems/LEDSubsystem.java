// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;

public class LEDSubsystem extends SubsystemBase {
  private AddressableLED ledStrip = new AddressableLED(9);
  private AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(120);

  /** Creates a new LEDSubsystem. */
  public LEDSubsystem() {
    ledStrip.setColorOrder(AddressableLED.ColorOrder.kRGB);
    ledStrip.setLength(ledBuffer.getLength());
    ledStrip.setData(ledBuffer);
    ledStrip.start();

    setDefaultCommand(setLED(Constants.LEDConstants.scrollingGradientNavyDarkGreen));
  }

  @Override
  public void periodic() { // This method will be called once per scheduler run
    // Set the LEDs
    ledStrip.setData(ledBuffer);
  }

  public Command setLED(LEDPattern pattern) {
    return run(
        () -> {
          pattern.applyTo(ledBuffer);
        });
  }
}
