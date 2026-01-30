package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
  private final VisionIO io;
  private final VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();
  private final AprilTagFieldLayout fieldLayout;

  public Vision(VisionIO io, AprilTagFieldLayout fieldLayout) {
    this.io = io;
    this.fieldLayout = fieldLayout;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Vision", inputs);

    Pose3d[] tagPoses = new Pose3d[inputs.trackedTags.length];
    for (int i = 0; i < tagPoses.length; i++) {
      tagPoses[i] = fieldLayout.getTagPose(inputs.trackedTags[i].id()).orElse(new Pose3d());
    }
    Logger.recordOutput("VisionTargets", tagPoses);
  }

  public Pose2d getPoseEstimate() {
    return inputs.latestPose;
  }

  public void seedImu() {
    io.seedImu();
  }

  public void assistImu() {
    io.assistImu();
  }
}

