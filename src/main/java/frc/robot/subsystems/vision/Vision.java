package frc.robot.subsystems.vision;

import static frc.robot.Constants.Vision.Settings.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;
import frc.robot.ShamLib.SMF.StateMachine;
import frc.robot.ShamLib.swerve.TimestampedPoseEstimator;
import frc.robot.ShamLib.vision.Limelight.Limelight;
import frc.robot.ShamLib.vision.PhotonVision.Apriltag.PVApriltagCam;
import java.util.*;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Vision extends StateMachine<Vision.State> {
  private final List<Consumer<List<TimestampedPoseEstimator.TimestampedVisionUpdate>>>
      visionUpdateConsumers = new ArrayList<>();
  private final List<Consumer<RingVisionUpdate>> ringVisionUpdateConsumers = new ArrayList<>();
  private final Limelight limelight;
  private final PVApriltagCam[] pvApriltagCams;

  Supplier<Pose2d> overallEstimateSupplier = null;

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
      cam.setMultiTagFallbackEstimationStrategy(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);

      applyPreAndPostProcesses(cam);
    }

    registerTransitions();
  }

  public void setOverallEstimateSupplier(Supplier<Pose2d> supplier) {
    overallEstimateSupplier = supplier;
  }

  private void applyPreAndPostProcesses(PVApriltagCam cam) {
    HashMap<Integer, Double[]> ambiguityAverages = new HashMap<>();
    int avgLength = 100;
    double ambiguityThreshold = 0.4;
    double distanceFromLastEstimateScalar = 2.0;

    cam.setPreProcess(
        (pipelineData) -> {
          if (!pipelineData.hasTargets()) return pipelineData;

          int idx = 0;

          for (var tag : pipelineData.getTargets()) {
            if (!ambiguityAverages.containsKey(tag.getFiducialId())) {
              Double[] arr = new Double[avgLength];
              Arrays.fill(arr, -1.0);
              arr[0] = tag.getPoseAmbiguity();

              ambiguityAverages.put(tag.getFiducialId(), arr);
            } else {
              var arr = ambiguityAverages.get(tag.getFiducialId());
              System.arraycopy(arr, 0, arr, 1, arr.length - 1);
              arr[0] = tag.getPoseAmbiguity();
            }

            double avg = 0;
            double count = 0;
            for (Double a : ambiguityAverages.get(tag.getFiducialId())) {
              if (a >= 0) {
                avg += a;
                count++;
              }
            }

            avg /= count;

            PhotonTrackedTarget target =
                new PhotonTrackedTarget(
                    tag.getYaw(),
                    tag.getPitch(),
                    tag.getArea(),
                    tag.getSkew(),
                    tag.getFiducialId(),
                    tag.getBestCameraToTarget(),
                    tag.getAlternateCameraToTarget(),
                    avg,
                    tag.getMinAreaRectCorners(),
                    tag.getDetectedCorners());

            pipelineData.targets.set(idx, target);

            Logger.recordOutput(
                "Vision/" + cam.getName() + "/target-" + target.getFiducialId() + "-avg-ambiguity",
                target.getPoseAmbiguity());

            idx++;
          }

          pipelineData.targets.removeIf(target -> target.getPoseAmbiguity() > ambiguityThreshold);

          return pipelineData;
        });

    cam.setPostProcess(
        (estimate) -> {
          var defaultProcess = cam.defaultPostProcess(estimate);

          if (overallEstimateSupplier != null) {
            return new TimestampedPoseEstimator.TimestampedVisionUpdate(
                defaultProcess.timestamp(),
                defaultProcess.pose(),
                defaultProcess
                    .stdDevs()
                    .times(
                        overallEstimateSupplier
                                .get()
                                .getTranslation()
                                .getDistance(defaultProcess.pose().getTranslation())
                            * distanceFromLastEstimateScalar));
          } else {
            return defaultProcess;
          }
        });
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
    addOmniTransition(State.TRAP);
  }

  private List<TimestampedPoseEstimator.TimestampedVisionUpdate> getLatestVisionUpdates() {
    List<TimestampedPoseEstimator.TimestampedVisionUpdate> updates = new ArrayList<>();

    boolean onlyOneForTrap = getState() == State.TRAP;

    for (PVApriltagCam cam : pvApriltagCams) {
      if (!onlyOneForTrap || cam.getName() == Constants.Vision.Settings.TRAP_CAMERA) {
        cam.getLatestEstimate()
            .ifPresent(
                (update) -> {
                  updates.add(update);

                  Logger.recordOutput("Vision/" + cam.getName() + "/latestEstimate", update.pose());
                });
      }
    }

    return updates;
  }

  @AutoLogOutput(key = "Vision/RingTarget")
  public Translation2d[] getCurrentRingTarget() {
    RingVisionUpdate update = getLatestRingVisionUpdate();

    if (update != null) {
      return new Translation2d[] {
        new Translation2d(update.centerOffsetX.getDegrees(), update.centerOffsetY.getDegrees())
      };

    } else return new Translation2d[] {};
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
    List<TimestampedPoseEstimator.TimestampedVisionUpdate> poseUpdates = getLatestVisionUpdates();
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
    TRAP,

    // FLAGS
    HAS_RING_TARGET,
    PV_INSTANCE_DISCONNECT
  }

  public record RingVisionUpdate(Rotation2d centerOffsetX, Rotation2d centerOffsetY, double size) {}
}
