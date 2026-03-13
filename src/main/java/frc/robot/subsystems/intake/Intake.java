package frc.robot.subsystems.intake;

public class Intake {
  private final IntakeRoller roller;
  private final IntakeDeploy deploy;

  public Intake(IntakeRollerIO roller, IntakeDeployIO deploy) {
    this.roller = new IntakeRoller(roller);
    this.deploy = new IntakeDeploy(deploy);
  }

  public IntakeRoller getRoller() {
    return roller;
  }

  public IntakeDeploy getDeploy() {
    return deploy;
  }
}

