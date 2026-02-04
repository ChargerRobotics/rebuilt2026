package frc.robot.subsystems.intake;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;

public class IntakeConstants {
  public static final DCMotor deployGearbox = DCMotor.getNEO(1);
  public static double deployReduction = 25 / 1;
  public static double deployEncoderPositionFactor = 2 * Math.PI / deployReduction;
  public static double deployEncoderVelocityFactor = (2 * Math.PI) / 60.0 / deployReduction;

  public static ArmFeedforward deployFeedforward = new ArmFeedforward(0, 0, 0);
  public static double deployKp = 0;
  public static double deployKd = 0;
  public static double deployKg = 0;
  public static double deployKs = 0;
  public static double deployKv = 0;
  public static double deployKa = 0;

  public static double deploySimKp = 0;
  public static double deploySimKd = 0;
  public static double deploySimKg = 0;
  public static double deploySimKs = 0;
  public static double deploySimKv = 0;
  public static double deploySimKa = 0;
}

