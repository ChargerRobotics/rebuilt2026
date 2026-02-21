package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.AngularVelocity;

public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    public double setpointRpm = 0;
    public boolean atSetpoint = true;
    public double rpm = 0;
    public double appliedVolts = 0;
    public double currentAmps = 0;
  }

  public default void updateInputs(ShooterIOInputs inputs) {}

  public default void setOpenLoop(double output) {}

  public default void setVelocity(AngularVelocity velocity) {}

  public default void stop() {}
}

