package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.ShamLib.PIDGains;
import frc.robot.ShamLib.swerve.SwerveDrive;
import frc.robot.ShamLib.swerve.SwerveSpeedLimits;
import frc.robot.subsystems.vision.Vision.RingVisionUpdate;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class AutoIntakeCommand extends Command {

  private final SwerveDrive drive;
  private final Supplier<RingVisionUpdate> visionUpdateSupplier;
  private final SwerveSpeedLimits speedLimits;

  private final PIDController thetaController;

  private final SlewRateLimiter xLimiter;
  private final SlewRateLimiter yLimiter;
  private final SlewRateLimiter thetaLimiter;

  public AutoIntakeCommand(
      SwerveDrive drive,
      Supplier<RingVisionUpdate> visionUpdateSupplier,
      SwerveSpeedLimits speedLimits,
      PIDGains thetaGains) {
    this.drive = drive;
    this.visionUpdateSupplier = visionUpdateSupplier;
    this.speedLimits = speedLimits;

    this.thetaController = thetaGains.applyToController();

    this.xLimiter = new SlewRateLimiter(speedLimits.getMaxAcceleration());
    this.yLimiter = new SlewRateLimiter(speedLimits.getMaxAcceleration());
    this.thetaLimiter = new SlewRateLimiter(speedLimits.getMaxAcceleration());
  }

  @Override
  public void initialize() {
    ChassisSpeeds speedsAtStart = drive.getChassisSpeeds();

    xLimiter.reset(speedsAtStart.vxMetersPerSecond);
    yLimiter.reset(speedsAtStart.vyMetersPerSecond);
    thetaLimiter.reset(speedsAtStart.omegaRadiansPerSecond);
  }

  @Override
  public void execute() {
    RingVisionUpdate currentRingData = visionUpdateSupplier.get();

    // Current ring data is non-null whenever there is a ring in the tracker
    if (currentRingData != null) {

      Pose3d cameraPose = Constants.Vision.Hardware.RING_CAMERA_POSE;

      double cameraHeight = cameraPose.getZ();

      double vertAngleToRing =
          currentRingData.centerOffsetY().getRadians() + cameraPose.getRotation().getY();

      // Mainly just extract the distance for logging purposes
      double distanceToRing = -cameraHeight / Math.tan(vertAngleToRing);

      double horizOffsetRadians = Math.PI / 2 + 0.25 * currentRingData.centerOffsetX().getRadians();

      // Camera relative pose of the ring
      Translation2d cameraRelativeEstimatedPose =
          new Translation2d(
              -Math.sin(horizOffsetRadians) * distanceToRing,
              Math.cos(horizOffsetRadians) * distanceToRing);

      // Where the ring should be relative to the bot
      Translation2d botRelativeEstimatedPose =
          new Translation2d(
              cameraRelativeEstimatedPose.getX() + cameraPose.getX(),
              cameraRelativeEstimatedPose.getY() + cameraPose.getY());

      Logger.recordOutput(
          "Vision/EstimatedRingPose",
          Constants.convertPose2dToPose3d(
              drive
                  .getPose()
                  .transformBy(new Transform2d(botRelativeEstimatedPose, new Rotation2d()))));

      // Extract chassis speeds from the translation
      double angleToMove =
          Math.atan2(botRelativeEstimatedPose.getY(), botRelativeEstimatedPose.getX());

      double targetSpeedX = speedLimits.getMaxSpeed() * Math.cos(angleToMove);
      double targetSpeedY = speedLimits.getMaxSpeed() * Math.sin(angleToMove);
      double targetRotation =
          thetaController.calculate(
              drive.getCurrentAngle().getRadians(),
              drive.getCurrentAngle().getRadians() + currentRingData.centerOffsetX().getRadians());

      ChassisSpeeds speeds =
          new ChassisSpeeds(
              xLimiter.calculate(targetSpeedX),
              yLimiter.calculate(targetSpeedY),
              thetaLimiter.calculate(targetRotation));

      drive.drive(speeds);
    } else {

    }
  }

  @Override
  public boolean isFinished() {
    return visionUpdateSupplier.get() == null;
  }

  @Override
  public void end(boolean interrupted) {
    drive.stopModules();
  }
}
