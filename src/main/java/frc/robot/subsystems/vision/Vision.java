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
  private List<Consumer<List<TimestampedPoseEstimator.TimestampedVisionUpdate>>>
      visionUpdateConsumers = new ArrayList<>();
  private List<Consumer<RingVisionUpdate>> ringVisionUpdateConsumers = new ArrayList<>();
  private final Limelight limelight;
  private final PVApriltagCam[] pvApriltagCams;

  public Vision(String limelight, Map<String, Pose3d> photonVisionInstances) {
    super("Vision", State.UNDETERMINED, State.class);

    this.limelight = new Limelight(limelight, Constants.currentBuildMode);

    pvApriltagCams =
        photonVisionInstances.entrySet()
            .stream().map(entry ->
                    new PVApriltagCam(
                        entry.getKey(),
                        Constants.currentBuildMode,
                        new Transform3d(new Pose3d(), entry.getValue()),
                        Constants.PhysicalConstants.APRIL_TAG_FIELD_LAYOUT))
            .toArray(PVApriltagCam[]::new);

    for (var cam : pvApriltagCams) {
      cam.setPoseEstimationStrategy(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);
      cam.setMultiTagFallbackEstimationStrategy(
          PhotonPoseEstimator.PoseStrategy.AVERAGE_BEST_TARGETS);
    }

    registerStateCommand();
    registerTransitions();
  }

  public void addVisionUpdateConsumers(
      Consumer<List<TimestampedPoseEstimator.TimestampedVisionUpdate>>... consumers) {
    visionUpdateConsumers.addAll(Arrays.stream(consumers).toList());
  }

  public void addRingVisionUpdateConsumers(Consumer<RingVisionUpdate>... consumers) {
    ringVisionUpdateConsumers.addAll(Arrays.stream(consumers).toList());
  }

  private void registerStateCommand() {
    registerStateCommand(State.ENABLED, enabledCommand());
  }

  private void registerTransitions() {
    addOmniTransition(State.ENABLED, () -> limelight.setPipeline(LIMELIGHT_NOTE_TRACK_PIPELINE));
    addOmniTransition(State.DISABLED);
  }

  private List<TimestampedPoseEstimator.TimestampedVisionUpdate> getLatestVisionUpdates() {
    List<TimestampedPoseEstimator.TimestampedVisionUpdate> updates = new ArrayList<>();

    for (PVApriltagCam cam : pvApriltagCams) {
      updates.add(cam.getLatestEstimate());
    }

    return updates;
  }

  private RingVisionUpdate getLatestRingVisionUpdate() {
    var inputs = limelight.getInputs();

    // if no target
    if (inputs.tv == 0) return null;

    return new RingVisionUpdate(Rotation2d.fromDegrees(inputs.tx), inputs.ta);
  }

  private Command enabledCommand() {
    return new RunCommand(
        () -> {
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

            for (var c : ringVisionUpdateConsumers) {
              c.accept(ringVisionUpdate);
            }
          } else {
            clearFlag(State.HAS_RING_TARGET);
          }

          if (Arrays.stream(pvApriltagCams).anyMatch((cam) -> !cam.isConnected())) {
            setFlag(State.PV_INSTANCE_DISCONNECT);
          } else {
            clearFlag(State.PV_INSTANCE_DISCONNECT);
          }
        });
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
  }

  public enum State {
    UNDETERMINED,
    ENABLED,
    DISABLED,

    // FLAGS
    HAS_RING_TARGET,
    PV_INSTANCE_DISCONNECT
  }

  public record RingVisionUpdate(Rotation2d centerOffset, double size) {}
}
