package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
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
    io.setMAXMotionPosition(Rotation2d.kZero);
    Logger.recordOutput("Intake/Deploy/Goal", 0);
  }

  public void retract() {
    io.setMAXMotionPosition(Rotation2d.fromDegrees(90));
    Logger.recordOutput("Intake/Deploy/Goal", Math.PI / 2);
  }
}

