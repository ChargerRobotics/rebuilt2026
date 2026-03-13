package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeDeploy;
import frc.robot.subsystems.intake.IntakeRoller;

public class IntakeCommands {
  private IntakeCommands() {}

  public static Command deploy(Intake intake) {
    IntakeDeploy deploy = intake.getDeploy();
    return Commands.startEnd(() -> deploy.deploy(), () -> deploy.retract(), deploy);
  }

  public static Command intake(Intake intake) {
    IntakeRoller roller = intake.getRoller();
    return Commands.startEnd(() -> roller.intake(), () -> roller.stop(), roller);
  }

  public static Command deployAndIntake(Intake intake) {
    IntakeRoller roller = intake.getRoller();
    IntakeDeploy deploy = intake.getDeploy();
    return Commands.startEnd(
      () -> {
        roller.intake();
        deploy.deploy();
      }, () -> {
        roller.stop();
        deploy.retract();
      },
      roller, deploy);
  }
}

