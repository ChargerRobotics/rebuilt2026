// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static frc.robot.subsystems.drive.DriveConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.PhoenixUtil;
import frc.robot.util.SparkUtil;

import java.util.Queue;
import java.util.function.DoubleSupplier;

/**
 * Module IO implementation for Spark Flex drive motor controller, Spark Max turn motor controller,
 * and duty cycle absolute encoder.
 */
public class ModuleIOReal implements ModuleIO {
  private final Rotation2d zeroRotation;

  // Hardware objects
  private final TalonFX driveTalon;
  private final SparkBase turnSpark;
  private final RelativeEncoder turnEncoder;
  private final CANcoder canCoder;

  private final StatusSignal<Angle> drivePosition;
  private final StatusSignal<AngularVelocity> driveVelocity;
  private final StatusSignal<Voltage> driveAppliedVolts;
  private final StatusSignal<Current> driveCurrent;

  private final VelocityVoltage velocityVoltageRequest = new VelocityVoltage(0);

  // Closed loop controllers
  private final SparkClosedLoopController turnController;

  // Queue inputs from odometry thread
  private final Queue<Double> timestampQueue;
  private final Queue<Double> drivePositionQueue;
  private final Queue<Double> turnPositionQueue;

  // Connection debouncers
  private final Debouncer driveConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Debouncer turnConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  public ModuleIOReal(int module) {
    zeroRotation = switch (module) {
      case 0 -> frontLeftZeroRotation;
      case 1 -> frontRightZeroRotation;
      case 2 -> backLeftZeroRotation;
      case 3 -> backRightZeroRotation;
      default -> Rotation2d.kZero;
    };
    driveTalon = new TalonFX(switch (module) {
      case 0 -> frontLeftDriveCanId;
      case 1 -> frontRightDriveCanId;
      case 2 -> backLeftDriveCanId;
      case 3 -> backRightDriveCanId;
      default -> 0;
    });
    turnSpark = new SparkMax(switch (module) {
      case 0 -> frontLeftTurnCanId;
      case 1 -> frontRightTurnCanId;
      case 2 -> backLeftTurnCanId;
      case 3 -> backRightTurnCanId;
      default -> 0;
    }, MotorType.kBrushless);
    canCoder = new CANcoder(switch (module) {
      case 0 -> frontLeftCANCoderId;
      case 1 -> frontRightCANCoderId;
      case 2 -> backLeftCANCoderId;
      case 3 -> backRightCANCoderId;
      default -> 0;
    });
    drivePosition = driveTalon.getPosition();
    driveVelocity = driveTalon.getVelocity();
    driveAppliedVolts = driveTalon.getMotorVoltage();
    driveCurrent = driveTalon.getStatorCurrent();
    turnEncoder = turnSpark.getEncoder();
    turnController = turnSpark.getClosedLoopController();

    // Configure drive motor
    // TODO: configure
    var driveConfig = new TalonFXConfiguration();
    driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    driveConfig.Slot0 = new Slot0Configs()
        .withKP(driveKp)
        .withKD(driveKd)
        .withKS(driveKs)
        .withKV(driveKv);
    driveConfig.Feedback.SensorToMechanismRatio = driveMotorReduction;
    driveConfig.CurrentLimits.StatorCurrentLimit = driveMotorCurrentLimit;
    driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    driveConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    PhoenixUtil.tryUntilOk(5, () -> driveTalon.getConfigurator().apply(driveConfig, 0.25));
    PhoenixUtil.tryUntilOk(5, () -> driveTalon.setPosition(0.0, 0.25));

    // Configure turn motor
    var turnConfig = new SparkMaxConfig();
    turnConfig
        .inverted(turnInverted)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(turnMotorCurrentLimit)
        .voltageCompensation(12.0);
    turnConfig
        .encoder
        .positionConversionFactor(turnEncoderPositionFactor)
        .velocityConversionFactor(turnEncoderVelocityFactor)
        .uvwAverageDepth(2);
    turnConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .positionWrappingEnabled(true)
        .positionWrappingInputRange(turnPIDMinInput, turnPIDMaxInput)
        .pid(turnKp, 0.0, turnKd);
    turnConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs((int) (1000.0 / odometryFrequency))
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    SparkUtil.tryUntilOk(turnSpark, 5, () -> turnSpark.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    SparkUtil.tryUntilOk(turnSpark, 5, () -> turnEncoder.setPosition(canCoder.getPosition().getValue().in(Radians)));

    // Create odometry queues
    timestampQueue = SparkOdometryThread.getInstance().makeTimestampQueue();
    drivePositionQueue =
        SparkOdometryThread.getInstance().registerSignal(drivePosition::getValueAsDouble);
    turnPositionQueue =
        SparkOdometryThread.getInstance().registerSignal(turnSpark, turnEncoder::getPosition);
    
    BaseStatusSignal.setUpdateFrequencyForAll(DriveConstants.odometryFrequency, drivePosition);
    ParentDevice.optimizeBusUtilizationForAll(driveTalon);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    StatusCode driveStatus = BaseStatusSignal.refreshAll(drivePosition, driveVelocity, driveAppliedVolts, driveCurrent);

    // Update drive inputs
    inputs.driveConnected = driveConnectedDebounce.calculate(driveStatus.isOK());
    inputs.drivePositionRad = drivePosition.getValue().in(Radians);
    inputs.driveVelocityRadPerSec = driveVelocity.getValue().in(RadiansPerSecond);
    inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
    inputs.driveCurrentAmps = driveCurrent.getValueAsDouble();

    // Update turn inputs
    SparkUtil.sparkStickyFault = false;
    SparkUtil.ifOk(
        turnSpark,
        turnEncoder::getPosition,
        (value) -> inputs.turnPosition = new Rotation2d(value).minus(zeroRotation));
    SparkUtil.ifOk(turnSpark, turnEncoder::getVelocity, (value) -> inputs.turnVelocityRadPerSec = value);
    SparkUtil.ifOk(
        turnSpark,
        new DoubleSupplier[] {turnSpark::getAppliedOutput, turnSpark::getBusVoltage},
        (values) -> inputs.turnAppliedVolts = values[0] * values[1]);
    SparkUtil.ifOk(turnSpark, turnSpark::getOutputCurrent, (value) -> inputs.turnCurrentAmps = value);
    inputs.turnConnected = turnConnectedDebounce.calculate(!SparkUtil.sparkStickyFault);

    // Update odometry inputs
    inputs.odometryTimestamps =
        timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryDrivePositionsRad =
        drivePositionQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryTurnPositions =
        turnPositionQueue.stream()
            .map((Double value) -> new Rotation2d(value).minus(zeroRotation))
            .toArray(Rotation2d[]::new);
    timestampQueue.clear();
    drivePositionQueue.clear();
    turnPositionQueue.clear();
  }

  @Override
  public void setDriveOpenLoop(double output) {
    driveTalon.setVoltage(output);
  }

  @Override
  public void setTurnOpenLoop(double output) {
    turnSpark.setVoltage(output);
  }

  @Override
  public void setDriveVelocity(double velocityRadPerSec) {
    double velocityRotPerSec = Units.radiansToRotations(velocityRadPerSec);
    driveTalon.setControl(velocityVoltageRequest.withVelocity(velocityRotPerSec));
  }

  @Override
  public void setTurnPosition(Rotation2d rotation) {
    double setpoint =
        MathUtil.inputModulus(
            rotation.plus(zeroRotation).getRadians(), turnPIDMinInput, turnPIDMaxInput);
    turnController.setSetpoint(setpoint, ControlType.kPosition);
  }
}
