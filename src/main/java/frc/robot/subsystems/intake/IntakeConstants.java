package frc.robot.subsystems.intake;

import edu.wpi.first.math.system.plant.DCMotor;

public class IntakeConstants {
  public static final DCMotor rollerGearbox = DCMotor.getNEO(1);
  public static final double rollerReduction = 4 / 1;

  public static final DCMotor deployGearbox = DCMotor.getNEO(1);
  public static final double deployReduction = 25 / 1;
  public static final double deployEncoderPositionFactor = 2 * Math.PI / deployReduction;
  public static final double deployEncoderVelocityFactor = (2 * Math.PI) / 60.0 / deployReduction;

  public static final int rollerCanId = 20;
  public static final int deployCanId = 21;

  public static final int rollerCurrentLimit = 20;
  public static final double rollerMaxAcceleration = 20000;
  public static final double rollerAllowedProfileError = 1000;

  public static final int deployCurrentLimit = 20;
  public static final double deployCruiseVelocity = 0.5 * Math.PI;
  public static final double deployMaxAcceleration = Math.PI;
  public static final double deployAllowedProfileError = 0.2 * Math.PI;

  public static final double deployKp = 0;
  public static final double deployKd = 0;
  public static final double deployKg = 0;
  public static final double deployKs = 0;
  public static final double deployKv = 0;
  public static final double deployKa = 0;

  public static final double deploySimKp = 5;
  public static final double deploySimKd = 0;
  public static final double deploySimKg = 0;
  public static final double deploySimKs = 0;
  public static final double deploySimKv = 0;
  public static final double deploySimKa = 0;
}

