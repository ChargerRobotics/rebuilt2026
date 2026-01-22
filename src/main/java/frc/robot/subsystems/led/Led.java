package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Led extends SubsystemBase {
  private final LedControlIO io;

  public Led(LedControlIO io) {
    this.io = io;
  }

  public void lightClimbing() {
    io.lightClimbing();
  }
}

