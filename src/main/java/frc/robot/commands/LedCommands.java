package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.led.Led;

public class LedCommands {
  private LedCommands() {}

  public static Command lightClimbing(Led led) {
    return Commands.runOnce(() -> led.lightClimbing(), led);
  }
}

