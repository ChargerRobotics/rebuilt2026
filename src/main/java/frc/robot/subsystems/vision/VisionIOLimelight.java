package frc.robot.subsystems.vision;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.util.LimelightHelpers;

public class VisionIOLimelight implements VisionIO {
  private final String name;
  private final Supplier<Rotation2d> headingSupplier;

  private IMUMode imuMode = IMUMode.UNKNOWN;

  public VisionIOLimelight(String name, Supplier<Rotation2d> headingSupplier) {
    this.name = name;
    this.headingSupplier = headingSupplier;
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    LimelightHelpers.SetRobotOrientation(name, headingSupplier.get().getDegrees(), 0, 0, 0, 0, 0);
    LimelightHelpers.PoseEstimate m2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
    inputs.latestPose = m2.pose;
    inputs.timestampSeconds = m2.timestampSeconds;
    inputs.avgTagDistance = m2.avgTagDist;

    TrackedTag[] trackedTags = new TrackedTag[m2.rawFiducials.length];
    for (int i = 0; i < m2.rawFiducials.length; i++) {
      trackedTags[i] = new TrackedTag(m2.rawFiducials[i].id, m2.rawFiducials[i].ambiguity);
    }

    inputs.trackedTags = trackedTags;
    inputs.imuMode = imuMode;
  }

  @Override
  public void seedImu() {
    LimelightHelpers.SetIMUMode(name, 1);
    imuMode = IMUMode.EXTERNAL_ONLY;
  }

  @Override
  public void assistImu() {
    LimelightHelpers.SetIMUMode(name, 4);
    imuMode = IMUMode.INTERNAL_EXTERNAL_ASSIST;
  }
}

