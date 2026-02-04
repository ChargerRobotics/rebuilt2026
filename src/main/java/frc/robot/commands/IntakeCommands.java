package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.IntakeDeploy;

public class IntakeCommands {
  private IntakeCommands() {}

  public static Command deploy(IntakeDeploy deploy) {
    return Commands.startEnd(
      () -> {
        deploy.deploy();
      }, () -> {
        deploy.retract();
      },
      deploy);
  }

  public static Command deployAndIntake(IntakeDeploy deploy) {
    return Commands.startEnd(
      () -> {
        deploy.deploy();
      }, () -> {
        deploy.retract();
      },
      deploy);
  }
}

