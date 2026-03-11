package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IntakeDeployIOSim implements IntakeDeployIO {
  // NOTE: use SparkMaxSim here?
  private final DCMotorSim sim;

  private final PIDController pidController = new PIDController(IntakeConstants.deploySimKp, 0.0, IntakeConstants.deploySimKd);
  private final TrapezoidProfile trapezoidProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(IntakeConstants.deployCruiseVelocity, IntakeConstants.deployMaxAcceleration));
  private final ArmFeedforward feedforward = new ArmFeedforward(IntakeConstants.deploySimKs, IntakeConstants.deploySimKg, IntakeConstants.deploySimKv, IntakeConstants.deploySimKa);

  private TrapezoidProfile.State goal = new TrapezoidProfile.State();
  private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();

  private boolean closedLoop = false;
  private double appliedVolts = 0;

  public IntakeDeployIOSim() {
    this.sim = new DCMotorSim(
      LinearSystemId.createSingleJointedArmSystem(IntakeConstants.deployGearbox, 2, IntakeConstants.deployReduction),
      IntakeConstants.deployGearbox
    );
  }

  @Override
  public void updateInputs(IntakeDeployIOInputs inputs) {
    if (closedLoop) {
      TrapezoidProfile.State next = trapezoidProfile.calculate(TimedRobot.kDefaultPeriod, setpoint, goal);
      double ff = feedforward.calculateWithVelocities(sim.getAngularPositionRad(), sim.getAngularVelocityRadPerSec(), next.velocity);
      appliedVolts = ff + pidController.calculate(sim.getAngularPositionRad(), next.position);
      setpoint = next;
    } else {
      pidController.reset();
      setpoint = new TrapezoidProfile.State();
    }

    sim.setInputVoltage(MathUtil.clamp(appliedVolts, -12, 12));
    sim.update(0.02);

    inputs.setpointRad = pidController.getSetpoint();
    inputs.positionRad = sim.getAngularPositionRad();
    inputs.velocityRadPerSec = sim.getAngularVelocityRadPerSec();
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = sim.getCurrentDrawAmps();
  }

  @Override
  public void setOpenLoop(double output) {
    closedLoop = false;
    appliedVolts = output;
  }

  @Override
  public void setMAXMotionPosition(Rotation2d setpoint) {
    closedLoop = true;
    goal.position = setpoint.getRadians();
  }
}

