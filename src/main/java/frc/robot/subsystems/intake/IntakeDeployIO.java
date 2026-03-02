package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface IntakeDeployIO {
  @AutoLog
  public static class IntakeDeployIOInputs {
    public Rotation2d setpoint = Rotation2d.kZero;
    public Rotation2d position = Rotation2d.kZero;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
  }

  public default void updateInputs(IntakeDeployIOInputs inputs) {}

  public default void setOpenLoop(double output) {}

  public default void setMAXMotionPosition(Rotation2d setpoint) {}
}

