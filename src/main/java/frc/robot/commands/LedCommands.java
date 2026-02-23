package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.led.Led;
import frc.robot.subsystems.led.RobotState;

public class LedCommands {
  private LedCommands() {}

  public static Command addState(Led led, RobotState state) {
    return Commands.startEnd(() -> led.addState(state), () -> led.removeState(state), led);
  }
}

