package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;

public interface VisionIO {
  @AutoLog
  public static class VisionIOInputs {
    public Pose2d latestPose = new Pose2d();
    public double timestampSeconds = 0;
    public double avgTagDistance = 0;
    public TrackedTag[] trackedTags = new TrackedTag[0];
    public IMUMode imuMode = IMUMode.UNKNOWN;
  }

  public static record TrackedTag(int id, double ambiguity) {}

  public static enum IMUMode {
    EXTERNAL_ONLY,
    EXTERNAL_SEED,
    INTERNAL_ONLY,
    INTERNAL_M1_ASSIST,
    INTERNAL_EXTERNAL_ASSIST,
    UNKNOWN,
  }

  public default void updateInputs(VisionIOInputs inputs) {}

  public default void seedImu() {}

  public default void assistImu() {}
}

