package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ShooterIOSim implements ShooterIO {
  private final DCMotorSim sim;

  private final PIDController pidController = new PIDController(ShooterConstants.simKp, ShooterConstants.simKi, ShooterConstants.simKd);
  
  private boolean closedLoop = false;
  private double setpointRpm = 0;
  private double appliedVolts = 0;

  public ShooterIOSim() {
    this.sim = new DCMotorSim(
      LinearSystemId.createDCMotorSystem(ShooterConstants.gearbox, 0.05, ShooterConstants.gearStepup),
      ShooterConstants.gearbox
    );
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    if (closedLoop) {
      appliedVolts = pidController.calculate(sim.getAngularVelocityRPM());
    } else {
      pidController.reset();
    }

    sim.setInputVoltage(MathUtil.clamp(appliedVolts, -12, 12));
    sim.update(0.02);

    inputs.setpointRpm = setpointRpm;
    inputs.atSetpoint = MathUtil.isNear(setpointRpm, sim.getAngularVelocityRPM(), 10);
    inputs.rpm = sim.getAngularVelocityRPM();
    inputs.appliedVolts = 0;
    inputs.currentAmps = sim.getCurrentDrawAmps();
  }

  @Override
  public void setOpenLoop(double output) {
    closedLoop = false;
    appliedVolts = output;
  }

  @Override
  public void setVelocity(AngularVelocity velocity) {
    closedLoop = true;
    pidController.setSetpoint(velocity.in(RPM));
  }

  @Override
  public void stop() {
    closedLoop = true;
    appliedVolts = 0;
  }
}

