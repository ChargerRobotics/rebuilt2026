package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeDeploy extends SubsystemBase {
  private final IntakeDeployIO io;
  private final IntakeDepoyIOInputsAutoLogged inputs = new IntakeDepoyIOInputsAutoLogged();

  public IntakeDeploy(IntakeDeployIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake/Deploy", inputs);
  }

  public void deploy() {
    io.setMAXMotionPosition(0);
    Logger.recordOutput("Intake/Deploy/Goal", 0);
  }

  public void retract() {
    io.setMAXMotionPosition(Math.PI / 2);
    Logger.recordOutput("Intake/Deploy/Goal", Math.PI / 2);
  }
}

