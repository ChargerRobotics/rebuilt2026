package frc.robot.subsystems.led;

import java.util.TreeSet;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.signals.RGBWColor;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Led extends SubsystemBase {
  private static final SolidColor DEFAULT_ANIMATION = new SolidColor(8, 399).withColor(new RGBWColor(22, 51, 85));
  private final LedControlIO io;
  private final LedControlIOInputsAutoLogged inputs = new LedControlIOInputsAutoLogged();

  private final TreeSet<RobotState> states = new TreeSet<>();

  public Led(LedControlIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Led", inputs);
  }

  public void addState(RobotState state) {
    states.add(state);
    updateAnimation();
  }

  public void removeState(RobotState state) {
    states.remove(state);
    updateAnimation();
  }

  public void updateAnimation() {
    ControlRequest animation = states.size() == 0 ? DEFAULT_ANIMATION : states.first().getAnimation();
    io.request(animation);
  }
}

