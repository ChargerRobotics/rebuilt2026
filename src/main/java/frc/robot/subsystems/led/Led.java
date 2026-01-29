package frc.robot.subsystems.led;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Led extends SubsystemBase {
  private final LedControlIO io;
  private final LedControlIOInputsAutoLogged inputs = new LedControlIOInputsAutoLogged();

  public Led(LedControlIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Led", inputs);
  }

  public void lightClimbing() {
    io.lightClimbing();
  }
}

