package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ShamLib.PIDGains;
import frc.robot.ShamLib.swerve.SwerveDrive;
import frc.robot.ShamLib.swerve.SwerveSpeedLimits;
import frc.robot.subsystems.vision.Vision.RingVisionUpdate;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class AutoIntakeCommand extends Command {

  private final SwerveDrive drive;
  private final Supplier<RingVisionUpdate> visionUpdateSupplier;
  private final SwerveSpeedLimits speedLimits;

  private final PIDController thetaController;

  private final SlewRateLimiter xLimiter;
  private final SlewRateLimiter yLimiter;
  private final SlewRateLimiter thetaLimiter;

  private final BooleanSupplier intakeProxTripped;
  private final BooleanSupplier indexerReceivedRing;

  private final double lostTargetTimeoutTime;

  private Timer lostTargetTimeout = new Timer();

  private boolean end = false;

  public AutoIntakeCommand(
      SwerveDrive drive,
      Supplier<RingVisionUpdate> visionUpdateSupplier,
      SwerveSpeedLimits speedLimits,
      PIDGains thetaGains,
      BooleanSupplier intakeProxTripped,
      double lostTargetTimeoutTime,
      BooleanSupplier indexerReceivedRing) {
    this.drive = drive;
    this.visionUpdateSupplier = visionUpdateSupplier;
    this.speedLimits = speedLimits;

    this.intakeProxTripped = intakeProxTripped;
    this.indexerReceivedRing = indexerReceivedRing;

    this.thetaController = thetaGains.applyToController();

    this.xLimiter = new SlewRateLimiter(speedLimits.getMaxAcceleration());
    this.yLimiter = new SlewRateLimiter(speedLimits.getMaxAcceleration());
    this.thetaLimiter = new SlewRateLimiter(speedLimits.getMaxAcceleration());

    this.lostTargetTimeoutTime = lostTargetTimeoutTime;
  }

  @Override
  public void initialize() {
    ChassisSpeeds speedsAtStart = drive.getChassisSpeeds();

    xLimiter.reset(speedsAtStart.vxMetersPerSecond);
    yLimiter.reset(speedsAtStart.vyMetersPerSecond);
    thetaLimiter.reset(speedsAtStart.omegaRadiansPerSecond);

    lostTargetTimeout.stop();
    lostTargetTimeout.reset();

    end = false;
  }

  @Override
  public void execute() {
    RingVisionUpdate currentRingData = visionUpdateSupplier.get();

    // Current ring data is non-null whenever there is a ring in the tracker
    if (currentRingData != null) {

      double targetRotation =
          thetaController.calculate(
              drive.getCurrentAngle().getRadians(),
              drive.getCurrentAngle().getRadians()
                  - 0.5 * currentRingData.centerOffsetX().getRadians());

      ChassisSpeeds speeds =
          new ChassisSpeeds(
              xLimiter.calculate(-speedLimits.getMaxSpeed()),
              yLimiter.calculate(0),
              thetaLimiter.calculate(targetRotation));

      drive.drive(speeds);
    } else {
      // Either we've lost the target or will see the ring in the intake shortly; start the timeout
      // clock
      if (lostTargetTimeout.get() == 0) lostTargetTimeout.start();

      if (!lostTargetTimeout.hasElapsed(lostTargetTimeoutTime)
          || intakeProxTripped.getAsBoolean()) {
        // Continue driving to wait for prox
        drive.drive(
            new ChassisSpeeds(
                xLimiter.calculate(-speedLimits.getMaxSpeed()),
                yLimiter.calculate(0),
                thetaLimiter.calculate(0)));
      } else {
        // We've failed to get a ring
        end = true;
      }
    }
  }

  @Override
  public boolean isFinished() {
    return end || indexerReceivedRing.getAsBoolean();
  }

  @Override
  public void end(boolean interrupted) {
    // drive.stopModules();
    drive.drive(new ChassisSpeeds(0, 0, 0));

    lostTargetTimeout.stop();
  }
}
