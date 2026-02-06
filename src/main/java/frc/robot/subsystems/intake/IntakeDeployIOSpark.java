package frc.robot.subsystems.intake;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.util.SparkUtil;

public class IntakeDeployIOSpark implements IntakeDeployIO {
  private final SparkMax sparkMax;
  private final RelativeEncoder encoder;
  private final SparkClosedLoopController closedLoopController;

  public IntakeDeployIOSpark() {
    this.sparkMax = new SparkMax(IntakeConstants.deployCanId, MotorType.kBrushless);
    this.encoder = sparkMax.getEncoder();
    this.closedLoopController = sparkMax.getClosedLoopController();

    SparkMaxConfig config = new SparkMaxConfig();
    config
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(IntakeConstants.deployCurrentLimit)
      .voltageCompensation(12);
    config
      .encoder
      .positionConversionFactor(IntakeConstants.deployEncoderPositionFactor);
    config
      .closedLoop
      .maxMotion
      .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal)
      .cruiseVelocity(IntakeConstants.deployCruiseVelocity)
      .maxAcceleration(IntakeConstants.deployMaxAcceleration)
      .allowedProfileError(IntakeConstants.deployAllowedProfileError);
    config
      .closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .pid(IntakeConstants.deployKp, 0.0, IntakeConstants.deployKd);
    config
      .closedLoop
      .feedForward
      .kS(IntakeConstants.deployKs)
      .kV(IntakeConstants.deployKv)
      .kA(IntakeConstants.deployKa)
      .kCos(IntakeConstants.deployKg)
      .kCosRatio(1 / (2 * Math.PI));
    SparkUtil.tryUntilOk(sparkMax, 5, () -> sparkMax.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(IntakeDeployIOInputs inputs) {
    inputs.position = Rotation2d.fromRadians(encoder.getPosition());
    inputs.appliedVolts = sparkMax.getAppliedOutput() * sparkMax.getBusVoltage();
    inputs.currentAmps = sparkMax.getOutputCurrent();
  }

  @Override
  public void setOpenLoop(double output) {
    sparkMax.setVoltage(output);
  }

  @Override
  public void setMAXMotionPosition(Rotation2d setpoint) {
    closedLoopController.setSetpoint(setpoint.getRadians(), ControlType.kMAXMotionPositionControl);
  }
}

