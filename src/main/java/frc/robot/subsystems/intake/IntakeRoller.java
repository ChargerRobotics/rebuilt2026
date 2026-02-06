package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

public class IntakeRoller {
  private final IntakeRollerIO io;
  private final IntakeRollerIOInputsAutoLogged inputs = new IntakeRollerIOInputsAutoLogged();

  public IntakeRoller(IntakeRollerIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake/Roller", inputs);
  }

  public void intake() {
    io.setOpenLoop(0.8);
  }

  public void stop() {
    io.setOpenLoop(0);
  }
}

