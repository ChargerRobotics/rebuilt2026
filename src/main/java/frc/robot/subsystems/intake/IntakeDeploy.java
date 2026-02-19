package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;

public class IntakeDeploy {
  private final IntakeDeployIO io;
  private final IntakeDeployIOInputsAutoLogged inputs = new IntakeDeployIOInputsAutoLogged();

  public IntakeDeploy(IntakeDeployIO io) {
    this.io = io;
    retract();
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake/Deploy", inputs);
  }

  public void runCharacterization(double output) {
    io.setOpenLoop(output);
  }

  public void deploy() {
    io.setMAXMotionPosition(Rotation2d.kZero);
  }

  public void retract() {
    io.setMAXMotionPosition(Rotation2d.fromDegrees(90));
  }
}

