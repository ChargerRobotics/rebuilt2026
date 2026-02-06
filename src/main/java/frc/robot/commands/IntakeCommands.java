package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.Intake;

public class IntakeCommands {
  private IntakeCommands() {}

  public static Command deploy(Intake intake) {
    return Commands.startEnd(
      () -> {
        intake.deploy();
      }, () -> {
        intake.retract();
      },
      intake);
  }

  public static Command deployAndIntake(Intake intake) {
    return Commands.startEnd(
      () -> {
        intake.intake();
        intake.deploy();
      }, () -> {
        intake.stopIntake();
        intake.retract();
      },
      intake);
  }
}

