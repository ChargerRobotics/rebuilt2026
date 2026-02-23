package frc.robot.subsystems.led;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.hardware.CANdle;

public class LedControlIOCANdle implements LedControlIO {
  private final CANdle candle;

  public LedControlIOCANdle(int id) {
    this.candle = new CANdle(id);
  }

  @Override
  public void updateInputs(LedControlIOInputs inputs) {
    inputs.currentAnimation = candle.getAppliedControl().getName();
  }

  @Override
  public void request(ControlRequest request) {
    candle.setControl(request);
  }
}

