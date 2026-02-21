package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RPM;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  public Shooter(ShooterIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
  }

  public boolean atSetpoint() {
    return inputs.atSetpoint;
  }

  public void runCharacterization(double output) {
    io.setOpenLoop(output);
  }

  public void spinUp() {
    io.setVelocity(RPM.of(4500));
  }

  public void stop() {
    io.stop();
  }
}

