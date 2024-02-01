// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.LEDs.Color;
import frc.robot.subsystems.LEDs.LEDSegment;

public class SetLEDColor extends Command {
  private LEDs m_leds;
  private Color LEDColor;
  // private LEDSegment LEDSegment = new LedSegment;
  public SetLEDColor(LEDs leds, Color color) {
    m_leds = leds;
    LEDColor = color;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_leds);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    LEDSegment.MainStrip.setColor(LEDColor);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
