package frc.robot.subsystems.vision;

import frc.robot.Constants;
import frc.robot.ShamLib.swerve.TimestampedPoseEstimator;
import frc.robot.ShamLib.vision.Limelight.Limelight;
import frc.robot.ShamLib.vision.PhotonVision.Apriltag.PVApriltagCam;

import java.util.Arrays;
import java.util.function.Consumer;

public class Vision {
    private Consumer<TimestampedPoseEstimator.TimestampedVisionUpdate> visionUpdateConsumer = null;
    private final Limelight limelight;
    private final PVApriltagCam[] pvApriltagCams;

    public Vision(String limelight, String... photonVisionInstances) {
        this.limelight = new Limelight(limelight, Constants.currentBuildMode);

        pvApriltagCams = Arrays.stream(photonVisionInstances)
                .map((id) -> new PVApriltagCam(id, Constants.currentBuildMode, Constants.PhysicalConstants.APRIL_TAG_FIELD_LAYOUT))
                .toArray(PVApriltagCam[]::new);
    }
}
