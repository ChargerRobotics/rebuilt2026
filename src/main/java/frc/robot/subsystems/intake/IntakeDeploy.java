package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class IntakeDeploy extends SubsystemBase {
  private final IntakeDeployIO io;
  private final IntakeDepoyIOInputsAutoLogged inputs = new IntakeDepoyIOInputsAutoLogged();

  private final SysIdRoutine sysId;

  public IntakeDeploy(IntakeDeployIO io) {
    this.io = io;
    retract();

    sysId = new SysIdRoutine(
      new SysIdRoutine.Config(
        null,
        null,
        null,
        state -> Logger.recordOutput("Intake/Deploy/SysIdState", state.toString())
      ),
      new SysIdRoutine.Mechanism(voltage -> io.setOpenLoop(voltage.in(Volts)), null, this)
    );
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake/Deploy", inputs);
  }

  public void deploy() {
    io.setMAXMotionPosition(Rotation2d.kZero);
    Logger.recordOutput("Intake/Deploy/Goal", 0.0);
  }

  public void retract() {
    io.setMAXMotionPosition(Rotation2d.fromDegrees(90));
    Logger.recordOutput("Intake/Deploy/Goal", Math.PI / 2);
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> io.setOpenLoop(0))
      .withTimeout(1)
      .andThen(sysId.quasistatic(direction));
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> io.setOpenLoop(0))
      .withTimeout(1)
      .andThen(sysId.dynamic(direction));
  }
}

