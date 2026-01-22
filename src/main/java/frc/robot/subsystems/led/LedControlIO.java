package frc.robot.subsystems.led;

import org.littletonrobotics.junction.AutoLog;

public interface LedControlIO {
  @AutoLog
  public static class LedControlIOInputs {
    public String currentAnimation;
  }

  public default void updateInputs(LedControlIOInputs inputs) {}

  public default void lightClimbing() {}
}

