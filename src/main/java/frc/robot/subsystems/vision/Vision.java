package frc.robot.subsystems.vision;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.ShamLib.SMF.StateMachine;
import frc.robot.ShamLib.swerve.TimestampedPoseEstimator;
import frc.robot.ShamLib.vision.Limelight.Limelight;
import frc.robot.ShamLib.vision.PhotonVision.Apriltag.PVApriltagCam;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.Consumer;

public class Vision extends StateMachine<Vision.State> {
    private List<Consumer<List<TimestampedPoseEstimator.TimestampedVisionUpdate>>> visionUpdateConsumers = new ArrayList<>();
    private List<Consumer<RingVisionUpdate>> ringVisionUpdateConsumers = new ArrayList<>();
    private final Limelight limelight;
    private final PVApriltagCam[] pvApriltagCams;

    public Vision(String limelight, String... photonVisionInstances) {
        super("Vision", State.UNDETERMINED, State.class);

        this.limelight = new Limelight(limelight, Constants.currentBuildMode);

        pvApriltagCams = Arrays.stream(photonVisionInstances)
                .map((id) -> new PVApriltagCam(id, Constants.currentBuildMode, Constants.PhysicalConstants.APRIL_TAG_FIELD_LAYOUT))
                .toArray(PVApriltagCam[]::new);
    }

    public void addVisionUpdateConsumers(Consumer<List<TimestampedPoseEstimator.TimestampedVisionUpdate>>... consumers) {
        visionUpdateConsumers.addAll(Arrays.stream(consumers).toList());
    }

    public void addRingVisionUpdateConsumers(Consumer<RingVisionUpdate>... consumers) {
        ringVisionUpdateConsumers.addAll(Arrays.stream(consumers).toList());
    }

    private void registerStateCommand() {

    }

    private void registerTransitions() {

    }

    private List<TimestampedPoseEstimator.TimestampedVisionUpdate> getLatestVisionUpdates() {
        List<TimestampedPoseEstimator.TimestampedVisionUpdate> updates = new ArrayList<>();

        for (PVApriltagCam cam : pvApriltagCams) {
            updates.addAll(cam.getAllEstimates());
        }

        return updates;
    }

    private RingVisionUpdate getLatestRingVisionUpdate() {
        //TODO: implement ring vision on limelight
        return new RingVisionUpdate(new ArrayList<>());
    }

    private Command enabledCommand() {
        return new RunCommand(() -> {
            List<TimestampedPoseEstimator.TimestampedVisionUpdate> poseUpdates = getLatestVisionUpdates();
            RingVisionUpdate ringVisionUpdate = getLatestRingVisionUpdate();

            if (!poseUpdates.isEmpty()) {
                for (var c : visionUpdateConsumers) {
                    c.accept(poseUpdates);
                }
            }

            if (!ringVisionUpdate.targets().isEmpty()) {
                setFlag(State.HAS_RING_TARGET);

                for (var c : ringVisionUpdateConsumers) {
                    c.accept(ringVisionUpdate);
                }
            }
            else {
                clearFlag(State.HAS_RING_TARGET);
            }

            if (Arrays.stream(pvApriltagCams).anyMatch((cam) -> !cam.isConnected())) {
                setFlag(State.PV_INSTANCE_DISCONNECT);
            }
            else {
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

        //FLAGS
        HAS_RING_TARGET,
        PV_INSTANCE_DISCONNECT
    }

    public record RingVisionUpdate(List<Pair<Rotation2d, Transform2d>> targets) {}
}
