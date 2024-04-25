package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.ShamLib.PIDGains;
import frc.robot.ShamLib.swerve.SwerveDrive;
import frc.robot.ShamLib.swerve.SwerveSpeedLimits;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import java.util.function.UnaryOperator;

public class FacePointCommand extends Command {
  private final PIDController thetaController;

  private final Pose2d pose;
  private final Supplier<Pose2d> poseSupplier;

  private final SwerveDrive drivetrain;
  private final DoubleSupplier xSupplier;
  private final DoubleSupplier ySupplier;

  private final SlewRateLimiter xLimiter;
  private final SlewRateLimiter yLimiter;
  private final SlewRateLimiter thetaLimiter;

  private final Double maxLinearSpeed;

  private final double deadband;
  private final UnaryOperator<Double> controllerConversion;

  public FacePointCommand(
      SwerveDrive drivetrain,
      PIDGains holdGains,
      Pose2d pose,
      Supplier<Pose2d> poseSupplier,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      double deadband,
      UnaryOperator<Double> controllerConversion,
      Subsystem subsystem,
      SwerveSpeedLimits speedLimits) {
    this.pose = pose;
    this.poseSupplier = poseSupplier;

    this.drivetrain = drivetrain;
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;

    thetaController = new PIDController(holdGains.p, holdGains.i, holdGains.d);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    xLimiter = new SlewRateLimiter(speedLimits.getMaxAcceleration());
    yLimiter = new SlewRateLimiter(speedLimits.getMaxAcceleration());
    thetaLimiter = new SlewRateLimiter(speedLimits.getMaxRotationalAcceleration());

    maxLinearSpeed = speedLimits.getMaxSpeed();

    this.deadband = deadband;
    this.controllerConversion = controllerConversion;
  }

  public boolean atAngle(double tolerance) {
    thetaController.setTolerance(tolerance);
    return thetaController.atSetpoint();
  }

  @Override
  public void initialize() {
    resetSpeedLimiters();

    thetaController.setSetpoint(
        Constants.rotationBetween(poseSupplier.get(), pose)
            .minus(Constants.Drivetrain.Settings.SHOT_OFFSET)
            .getRadians());
  }

  @Override
  public void execute() {
    thetaController.setSetpoint(
        Constants.rotationBetween(poseSupplier.get(), pose)
            .minus(Constants.Drivetrain.Settings.SHOT_OFFSET)
            .getRadians());

    double correctedX = convertRawInput(xSupplier.getAsDouble()) * maxLinearSpeed;
    double correctedY = convertRawInput(ySupplier.getAsDouble()) * maxLinearSpeed;
    double correctedRot = thetaController.calculate(poseSupplier.get().getRotation().getRadians());

    ChassisSpeeds speeds;

    if (drivetrain.isFieldRelative()) {
      speeds =
          ChassisSpeeds.fromFieldRelativeSpeeds(
              correctedX, correctedY, correctedRot, drivetrain.getCurrentFieldOrientedAngle());
    } else {
      speeds = new ChassisSpeeds(correctedX, correctedY, correctedRot);
    }

    speeds.vxMetersPerSecond = xLimiter.calculate(speeds.vxMetersPerSecond);
    speeds.vyMetersPerSecond = yLimiter.calculate(speeds.vyMetersPerSecond);
    speeds.omegaRadiansPerSecond = thetaLimiter.calculate(speeds.omegaRadiansPerSecond);

    drivetrain.drive(speeds, maxLinearSpeed);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  private double convertRawInput(double rawInput) {
    double deadbandInput = deadband(rawInput, deadband);
    return controllerConversion.apply(Double.valueOf(deadbandInput));
  }

  private double deadband(double rawInput, double deadband) {
    if (Math.abs(rawInput) > deadband) {
      if (rawInput > 0.0) return (rawInput - deadband) / (1.0 - deadband);
      else return (rawInput + deadband) / (1.0 - deadband);
    } else return 0;
  }

  private void resetSpeedLimiters() {
    ChassisSpeeds currentSpeeds = drivetrain.getChassisSpeeds();
    xLimiter.reset(currentSpeeds.vxMetersPerSecond);
    yLimiter.reset(currentSpeeds.vyMetersPerSecond);
    thetaLimiter.reset(currentSpeeds.omegaRadiansPerSecond);
  }
}
