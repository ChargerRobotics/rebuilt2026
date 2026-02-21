package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RPM;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.util.SparkUtil;

public class ShooterIOSpark implements ShooterIO {
  private final SparkFlex sparkFlex;
  private final RelativeEncoder encoder;
  private final SparkClosedLoopController closedLoopController;

  public ShooterIOSpark() {
    this.sparkFlex = new SparkFlex(ShooterConstants.leftCanId, MotorType.kBrushless);
    this.encoder = sparkFlex.getEncoder();
    this.closedLoopController = sparkFlex.getClosedLoopController();
    SparkFlex right = new SparkFlex(ShooterConstants.rightCanId, MotorType.kBrushless);

    SparkFlexConfig rightConfig = new SparkFlexConfig();
    rightConfig.follow(sparkFlex);
    SparkUtil.tryUntilOk(right, 5, () -> right.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    SparkFlexConfig config = new SparkFlexConfig();
    config
      .idleMode(IdleMode.kCoast)
      .smartCurrentLimit(ShooterConstants.currentLimit)
      .voltageCompensation(12);
    config
      .closedLoop
      .allowedClosedLoopError(50, ClosedLoopSlot.kSlot0)
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .pid(ShooterConstants.kP, 0, ShooterConstants.kD);
    config
      .closedLoop
      .feedForward
      .sva(ShooterConstants.kS, ShooterConstants.kV, 0);
    SparkUtil.tryUntilOk(sparkFlex, 5, () -> sparkFlex.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.setpointRpm = closedLoopController.getSetpoint();
    inputs.atSetpoint = closedLoopController.isAtSetpoint();
    inputs.rpm = encoder.getVelocity();
    inputs.appliedVolts = sparkFlex.getAppliedOutput() * sparkFlex.getBusVoltage();
    inputs.currentAmps = sparkFlex.getOutputCurrent();
  }

  @Override
  public void setOpenLoop(double output) {
    sparkFlex.setVoltage(output);
  }

  @Override
  public void setVelocity(AngularVelocity velocity) {
    closedLoopController.setSetpoint(velocity.in(RPM), ControlType.kVelocity);
  }

  @Override
  public void stop() {
    closedLoopController.setSetpoint(0, ControlType.kVoltage);
  }
}

