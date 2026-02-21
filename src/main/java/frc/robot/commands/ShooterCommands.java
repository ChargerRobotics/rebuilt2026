package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.shooter.Shooter;

public class ShooterCommands {
  private ShooterCommands() {}
  
  public static Command shoot(Shooter shooter) {
    return Commands.runEnd(() -> shooter.spinUp(), () -> shooter.stop(), shooter);
  }
}

