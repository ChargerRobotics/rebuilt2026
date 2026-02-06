package frc.robot.subsystems.intake;

import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IntakeRollerIOSim implements IntakeRollerIO {
  private final DCMotorSim sim;

  public IntakeRollerIOSim() {
    this.sim = new DCMotorSim(
      LinearSystemId.createDCMotorSystem(IntakeConstants.rollerGearbox, 0.005, IntakeConstants.rollerReduction),
      IntakeConstants.rollerGearbox
    );
  }

  @Override
  public void updateInputs(IntakeRollerIOInputs inputs) {
    inputs.appliedVolts = sim.getInputVoltage();
    inputs.currentAmps = sim.getCurrentDrawAmps();
  }

  @Override
  public void setOpenLoop(double output) {
    sim.setInputVoltage(output);
  }
}

