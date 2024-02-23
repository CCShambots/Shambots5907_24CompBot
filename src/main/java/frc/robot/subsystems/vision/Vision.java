package frc.robot.subsystems.vision;

import static frc.robot.Constants.Vision.Settings.*;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants;
import frc.robot.ShamLib.SMF.StateMachine;
import frc.robot.ShamLib.swerve.TimestampedPoseEstimator;
import frc.robot.ShamLib.vision.Limelight.Limelight;
import frc.robot.ShamLib.vision.PhotonVision.Apriltag.PVApriltagCam;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Map;
import java.util.function.Consumer;
import org.photonvision.PhotonPoseEstimator;

public class Vision extends StateMachine<Vision.State> {
  private final List<Consumer<List<TimestampedPoseEstimator.TimestampedVisionUpdate>>>
      visionUpdateConsumers = new ArrayList<>();
  private final List<Consumer<RingVisionUpdate>> ringVisionUpdateConsumers = new ArrayList<>();
  private final Limelight limelight;
  private final PVApriltagCam[] pvApriltagCams;

  public Vision(String limelight, Map<String, Pose3d> photonVisionInstances) {
    super("Vision", State.UNDETERMINED, State.class);

    this.limelight = new Limelight(limelight, Constants.currentBuildMode);

    pvApriltagCams =
        photonVisionInstances.entrySet().stream()
            .map(
                entry ->
                    new PVApriltagCam(
                        entry.getKey(),
                        Constants.currentBuildMode,
                        new Transform3d(new Pose3d(), entry.getValue()),
                        Constants.PhysicalConstants.APRIL_TAG_FIELD_LAYOUT,
                        VISION_TRUST_CUTOFF))
            .toArray(PVApriltagCam[]::new);

    for (var cam : pvApriltagCams) {
      cam.setPoseEstimationStrategy(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);
      cam.setMultiTagFallbackEstimationStrategy(
          PhotonPoseEstimator.PoseStrategy.AVERAGE_BEST_TARGETS);
    }

    registerTransitions();
  }

  public void addVisionUpdateConsumers(
      Consumer<List<TimestampedPoseEstimator.TimestampedVisionUpdate>>... consumers) {
    visionUpdateConsumers.addAll(Arrays.stream(consumers).toList());
  }

  public void addRingVisionUpdateConsumers(Consumer<RingVisionUpdate>... consumers) {
    ringVisionUpdateConsumers.addAll(Arrays.stream(consumers).toList());
  }

  private void registerTransitions() {
    addOmniTransition(State.ENABLED, () -> limelight.setPipeline(LIMELIGHT_NOTE_TRACK_PIPELINE));
    addOmniTransition(State.DISABLED);
  }

  private List<TimestampedPoseEstimator.TimestampedVisionUpdate> getLatestVisionUpdates() {
    List<TimestampedPoseEstimator.TimestampedVisionUpdate> updates = new ArrayList<>();

    for (PVApriltagCam cam : pvApriltagCams) {
      cam.getLatestEstimate().ifPresent(updates::add);
    }

    return updates;
  }

  private RingVisionUpdate getLatestRingVisionUpdate() {
    var inputs = limelight.getInputs();

    // if no target
    if (inputs.tv == 0) return null;

    return new RingVisionUpdate(
        Rotation2d.fromDegrees(inputs.tx), Rotation2d.fromDegrees(inputs.ty), inputs.ta);
  }

  @Override
  protected void determineSelf() {
    setState(State.ENABLED);
  }

  @Override
  protected void update() {
    for (var cam : pvApriltagCams) {
      cam.update();
    }

    limelight.update();

    updateConsumers();
  }

  private void updateConsumers() {
    List<TimestampedPoseEstimator.TimestampedVisionUpdate> poseUpdates =
            getLatestVisionUpdates();
    RingVisionUpdate ringVisionUpdate = getLatestRingVisionUpdate();

    if (!poseUpdates.isEmpty()) {
      for (var c : visionUpdateConsumers) {
        c.accept(poseUpdates);
      }
    }

    if (ringVisionUpdate != null) {
      setFlag(State.HAS_RING_TARGET);
    } else {
      clearFlag(State.HAS_RING_TARGET);
    }

    for (var c : ringVisionUpdateConsumers) {
      c.accept(ringVisionUpdate);
    }

    if (Arrays.stream(pvApriltagCams).anyMatch((cam) -> !cam.isConnected())) {
      setFlag(State.PV_INSTANCE_DISCONNECT);
    } else {
      clearFlag(State.PV_INSTANCE_DISCONNECT);
    }
  }

  public double getLimelightLatency() {
    return limelight.getLatency();
  }

  public Rotation2d getLimelightTargetOffset() {
    RingVisionUpdate latest = getLatestRingVisionUpdate();

    if (latest != null) return latest.centerOffsetX;
    else return new Rotation2d();
  }

  public enum State {
    UNDETERMINED,
    ENABLED,
    DISABLED,

    // FLAGS
    HAS_RING_TARGET,
    PV_INSTANCE_DISCONNECT
  }

  public record RingVisionUpdate(Rotation2d centerOffsetX, Rotation2d centerOffsetY, double size) {}
}
