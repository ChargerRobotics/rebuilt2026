package frc.robot.subsystems.led;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.controls.ControlRequest;

public interface LedControlIO {
  @AutoLog
  public static class LedControlIOInputs {
    public String currentAnimation;
  }

  public default void updateInputs(LedControlIOInputs inputs) {}

  public default void request(ControlRequest request) {}
}

