package frc.robot.subsystems.led;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.EmptyControl;

public class LedControlIOSim implements LedControlIO {
  private ControlRequest request = new EmptyControl();

  @Override
  public void updateInputs(LedControlIOInputs inputs) {
    inputs.currentAnimation = request.getName();
  }

  @Override
  public void request(ControlRequest request) {
    this.request = request;
  }
}

