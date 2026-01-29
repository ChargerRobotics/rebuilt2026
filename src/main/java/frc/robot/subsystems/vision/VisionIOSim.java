package frc.robot.subsystems.vision;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;

public class VisionIOSim implements VisionIO {
  private final Supplier<Rotation2d> headingSupplier;

  private IMUMode imuMode = IMUMode.UNKNOWN;

  public VisionIOSim(Supplier<Rotation2d> headingSupplier) {
    this.headingSupplier = headingSupplier;
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.latestPose = new Pose2d(0, 0, headingSupplier.get());
    inputs.timestampSeconds = Timer.getFPGATimestamp();
    inputs.imuMode = imuMode;
  }

  @Override
  public void seedImu() {
      imuMode = IMUMode.EXTERNAL_ONLY;
  }

  @Override
  public void assistImu() {
      imuMode = IMUMode.INTERNAL_EXTERNAL_ASSIST;
  }
}

