package frc.robot.subsystems.intake;

import edu.wpi.first.math.system.plant.DCMotor;

public class IntakeConstants {
  public static final DCMotor rollerGearbox = DCMotor.getNEO(1);
  public static final double rollerReduction = 4 / 1;

  public static final DCMotor deployGearbox = DCMotor.getNEO(1);
  public static double deployReduction = 25 / 1;
  public static double deployEncoderPositionFactor = 2 * Math.PI / deployReduction;
  public static double deployEncoderVelocityFactor = (2 * Math.PI) / 60.0 / deployReduction;

  public static int rollerCanId = 20;
  public static int deployCanId = 21;

  public static int rollerCurrentLimit = 20;
  public static double rollerMaxAcceleration = 20000;
  public static double rollerAllowedProfileError = 1000;

  public static int deployCurrentLimit = 20;
  public static double deployCruiseVelocity = 0.5 * Math.PI;
  public static double deployMaxAcceleration = Math.PI;
  public static double deployAllowedProfileError = 0.2 * Math.PI;

  public static double deployKp = 0;
  public static double deployKd = 0;
  public static double deployKg = 0;
  public static double deployKs = 0;
  public static double deployKv = 0;
  public static double deployKa = 0;

  public static double deploySimKp = 5;
  public static double deploySimKd = 0;
  public static double deploySimKg = 0;
  public static double deploySimKs = 0;
  public static double deploySimKv = 0;
  public static double deploySimKa = 0;
}

