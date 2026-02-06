package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IntakeDeployIOSim implements IntakeDeployIO {
  // NOTE: use SparkMaxSim here?
  private final DCMotorSim sim;

  private final PIDController pidController = new PIDController(IntakeConstants.deploySimKp, 0.0, IntakeConstants.deploySimKd);

  private boolean closedLoop = false;
  private double appliedVolts = 0.0;

  public IntakeDeployIOSim() {
    this.sim = new DCMotorSim(
      // TODO: MOI?
      LinearSystemId.createDCMotorSystem(IntakeConstants.deployGearbox, 0.01, IntakeConstants.deployReduction),
      IntakeConstants.deployGearbox
    );
  }

  @Override
  public void updateInputs(IntakeDeployIOInputs inputs) {
    if (closedLoop) {
      appliedVolts = pidController.calculate(sim.getAngularPositionRad());
    } else {
      pidController.reset();
    }

    sim.setInputVoltage(MathUtil.clamp(appliedVolts, -12, 12));
    sim.update(0.02);

    inputs.position = Rotation2d.fromRadians(sim.getAngularPositionRad());
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = sim.getCurrentDrawAmps();
  }

  @Override
  public void setOpenLoop(double output) {
    closedLoop = false;
    appliedVolts = output;
  }

  // TODO: setup feedforward
  @Override
  public void setMAXMotionPosition(Rotation2d setpoint) {
    closedLoop = true;
    pidController.setSetpoint(setpoint.getRadians());
  }
}

