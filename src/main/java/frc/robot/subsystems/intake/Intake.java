package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class Intake extends SubsystemBase {
  private final IntakeRoller roller;
  private final IntakeDeploy deploy;

  private final SysIdRoutine deploySysId;

  public Intake(IntakeRollerIO roller, IntakeDeployIO deploy) {
    this.roller = new IntakeRoller(roller);
    this.deploy = new IntakeDeploy(deploy);

    deploySysId = new SysIdRoutine(
      new SysIdRoutine.Config(
        null,
        null,
        null,
        state -> Logger.recordOutput("Intake/Deploy/SysIdState", state.toString())
      ),
      new SysIdRoutine.Mechanism(voltage -> roller.setOpenLoop(voltage.in(Units.Volts)), null, this)
    );
  }

  @Override
  public void periodic() {
    roller.periodic();
    deploy.periodic();
  }

  public void intake() {
    roller.intake();
  }

  public void stopIntake() {
    roller.stop();
  }

  public void deploy() {
    deploy.deploy();
  }

  public void retract() {
    deploy.retract();
  }

  public Command deploySysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> deploy.runCharacterization(0))
      .withTimeout(1)
      .andThen(deploySysId.quasistatic(direction));
  }

  public Command deploySysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> deploy.runCharacterization(0))
      .withTimeout(1)
      .andThen(deploySysId.dynamic(direction));
  }
}

