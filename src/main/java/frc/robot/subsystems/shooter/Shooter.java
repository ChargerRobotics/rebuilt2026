package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class Shooter extends SubsystemBase {
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  private final SysIdRoutine sysIdRoutine;

  public Shooter(ShooterIO io) {
    this.io = io;

    this.sysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(
        null,
        null,
        null,
        state -> Logger.recordOutput("Shooter/SysIdState", state.toString())
      ),
      new SysIdRoutine.Mechanism(voltage -> io.setOpenLoop(voltage.in(Volts)), null, this)
    );
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
    io.setVelocity(RPM.of(2000));
  }

  public void stop() {
    io.stop();
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.dynamic(direction);
  }
}

