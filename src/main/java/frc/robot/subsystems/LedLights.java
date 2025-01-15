// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CANConstants;
import frc.robot.Constants;

public class LedLights extends SubsystemBase {
  private CANdle m_candle = new CANdle(CANConstants.CANDLE_ID, Constants.CANIVORE_BUS_NAME);

  public LedLights() {}

  /**
   * Apply color to full set of LEDs on the robot
   *
   * @return a Command for applying a color
   */
  public Command applyColor(int r, int g, int b) {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
      () -> {
        m_candle.setLEDs(r,g,b);
      });
  }
}
