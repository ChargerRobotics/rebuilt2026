package frc.robot.subsystems.led;

import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.hardware.CANdle;

public class LedControlIOCANdle implements LedControlIO {
  private final CANdle candle;

  private final RainbowAnimation climbAnimation = new RainbowAnimation(8, 399);

  private String currentAnimation;

  public LedControlIOCANdle(CANdle candle) {
    this.candle = candle;
  }

  @Override
  public void updateInputs(LedControlIOInputs inputs) {
    inputs.currentAnimation = currentAnimation;
  }

  @Override
  public void lightClimbing() {
    currentAnimation = climbAnimation.getName();
    candle.setControl(climbAnimation);
  }
}

