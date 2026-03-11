package frc.robot.subsystems.shooter;

import edu.wpi.first.math.system.plant.DCMotor;

public class ShooterConstants {
  public static final DCMotor gearbox = DCMotor.getNeoVortex(2);
  public static final double gearStepup = 1;

  public static final int leftCanId = 30;
  public static final int rightCanId = 31;

  public static final int currentLimit = 80;

  public static final double kP = 0;
  public static final double kD = 0;
  public static final double kS = 0;
  public static final double kV = 0;

  public static final double simKp = 0.002605;
  public static final double simKd = 0;
  public static final double simKs = 0;
  public static final double simKv = 0.0017387;
}

