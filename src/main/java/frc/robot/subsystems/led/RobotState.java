package frc.robot.subsystems.led;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.FireAnimation;
import com.ctre.phoenix6.controls.LarsonAnimation;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.SingleFadeAnimation;
import com.ctre.phoenix6.signals.LarsonBounceValue;
import com.ctre.phoenix6.signals.RGBWColor;

public enum RobotState {
  INTAKING(new SingleFadeAnimation(8, 399).withColor(new RGBWColor(22, 51, 85))),
  AIMING(new FireAnimation(8, 399)),
  SHOOTING(new RainbowAnimation(8, 399)),
  CLIMBING(new LarsonAnimation(8, 399)
    .withColor(new RGBWColor(22, 51, 85))
    .withSize(5)
    .withBounceMode(LarsonBounceValue.Front)),
  ;

  private final ControlRequest animation;

  RobotState(ControlRequest animation) {
    this.animation = animation;
  }

  public ControlRequest getAnimation() {
      return animation;
  }
}

