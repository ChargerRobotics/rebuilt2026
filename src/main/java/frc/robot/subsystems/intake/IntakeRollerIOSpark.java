package frc.robot.subsystems.intake;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.util.SparkUtil;

public class IntakeRollerIOSpark implements IntakeRollerIO {
  private final SparkMax sparkMax;

  public IntakeRollerIOSpark() {
    this.sparkMax = new SparkMax(IntakeConstants.rollerCanId, MotorType.kBrushless);

    SparkMaxConfig config = new SparkMaxConfig();
    config
      .idleMode(IdleMode.kCoast)
      .smartCurrentLimit(IntakeConstants.rollerCurrentLimit)
      .voltageCompensation(12);
    SparkUtil.tryUntilOk(sparkMax, 5, () -> sparkMax.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(IntakeRollerIOInputs inputs) {
    inputs.appliedVolts = sparkMax.getAppliedOutput() * sparkMax.getBusVoltage();
    inputs.currentAmps = sparkMax.getOutputCurrent();
  }

  @Override
  public void setOpenLoop(double output) {
    sparkMax.setVoltage(output);
  }
}

