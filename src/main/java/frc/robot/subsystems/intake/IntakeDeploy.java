package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class IntakeDeploy extends SubsystemBase {
  private final IntakeDeployIO io;
  private final IntakeDeployIOInputsAutoLogged inputs = new IntakeDeployIOInputsAutoLogged();

  private final SysIdRoutine sysId;

  public IntakeDeploy(IntakeDeployIO io) {
    this.io = io;
    // retract();

    this.sysId = new SysIdRoutine(
      new SysIdRoutine.Config(
        null,
        null,
        null,
        state -> Logger.recordOutput("Intake/Deploy/SysIdState", state.toString())
      ),
      new SysIdRoutine.Mechanism(voltage -> runCharacterization(voltage.in(Volts)), null, this)
    );
  }

  @Override
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

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0))
      .withTimeout(1)
      .andThen(sysId.quasistatic(direction));
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0))
      .withTimeout(1)
      .andThen(sysId.dynamic(direction));
  }
}

