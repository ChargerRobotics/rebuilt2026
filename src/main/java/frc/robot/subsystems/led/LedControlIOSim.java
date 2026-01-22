package frc.robot.subsystems.led;

public class LedControlIOSim implements LedControlIO {
  private String currentState;

  @Override
  public void updateInputs(LedControlIOInputs inputs) {
    inputs.currentAnimation = currentState;
  }

  @Override
  public void lightClimbing() {
  }
}

