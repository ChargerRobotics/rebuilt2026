package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeRollerIO {
  @AutoLog
  public static class IntakeRollerIOInputs {
    public double appliedVolts = 0;
    public double currentAmps = 0;
  }

  public default void updateInputs(IntakeRollerIOInputs inputs) {}

  public default void setOpenLoop(double output) {}
}

