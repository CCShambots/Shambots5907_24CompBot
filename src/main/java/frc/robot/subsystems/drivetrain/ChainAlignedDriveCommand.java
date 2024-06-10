package frc.robot.subsystems.drivetrain;

import static frc.robot.Constants.PhysicalConstants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.ShamLib.AllianceManager;
import frc.robot.ShamLib.PIDGains;
import frc.robot.ShamLib.swerve.SwerveDrive;
import frc.robot.ShamLib.swerve.SwerveSpeedLimits;
import frc.robot.ShamLib.util.Line;
import frc.robot.util.StageSide;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import java.util.function.UnaryOperator;

public class ChainAlignedDriveCommand extends Command {

  private final SwerveDrive drive;
  private final Supplier<StageSide> targetStageSideSupplier;
  private final DoubleSupplier xSupplier;
  private final PIDController yController;
  private final PIDController xController;
  private final SwerveSpeedLimits speedLimits;

  private final PIDController thetaController;

  private final SlewRateLimiter xLimiter;
  private final SlewRateLimiter yLimiter;
  private final SlewRateLimiter thetaLimiter;

  private final double deadband;
  private final UnaryOperator<Double> controllerConversion;

  public ChainAlignedDriveCommand(
      SwerveDrive drive,
      PIDGains thetaGains,
      Supplier<StageSide> targetStageSideSupplier,
      DoubleSupplier xSupplier,
      PIDGains translationGains,
      double deadband,
      UnaryOperator<Double> controllerConversion,
      SwerveSpeedLimits speedLimits) {
    this.drive = drive;
    this.targetStageSideSupplier = targetStageSideSupplier;
    this.xSupplier = xSupplier;
    this.speedLimits = speedLimits;

    this.xController = translationGains.applyToController();
    this.yController = translationGains.applyToController();

    this.thetaController = thetaGains.applyToController();
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    this.xLimiter = new SlewRateLimiter(speedLimits.getMaxAcceleration());
    this.yLimiter = new SlewRateLimiter(speedLimits.getMaxAcceleration());
    this.thetaLimiter = new SlewRateLimiter(speedLimits.getMaxRotationalAcceleration());

    this.deadband = deadband;
    this.controllerConversion = controllerConversion;
  }

  @Override
  public void initialize() {
    ChassisSpeeds startingSpeeds = drive.getChassisSpeeds();

    xLimiter.reset(startingSpeeds.vxMetersPerSecond);
    yLimiter.reset(startingSpeeds.vyMetersPerSecond);
    thetaLimiter.reset(startingSpeeds.omegaRadiansPerSecond);
  }

  @Override
  public void execute() {
    Pose2d currentPose = drive.getPose();
    Rotation2d currentRotation = currentPose.getRotation();
    Rotation2d targetRotation = getTargetRotation();

    // Amount the driver is currently trying to move the bot
    double correctedX = convertRawInput(xSupplier.getAsDouble()) * speedLimits.getMaxSpeed();

    // Process how to center the bot on the chain

    Translation2d target =
        Line.fromPose2d(getTrapPose()).closestIntersection(currentPose.getTranslation());

    if (currentPose.getTranslation().getDistance(target) < Units.inchesToMeters(5)) {
      target = currentPose.getTranslation();
    }

    ChassisSpeeds fieldSpeeds =
        new ChassisSpeeds(
            xController.calculate(currentPose.getX(), target.getX()),
            yController.calculate(currentPose.getY(), target.getY()),
            thetaController.calculate(currentRotation.getRadians(), targetRotation.getRadians()));

    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(fieldSpeeds, currentRotation);

    // Only allow driver to move the bot towards the chain if the chassis is close to its target
    // rotation
    if (Constants.doubleEqual(
        currentRotation.minus(targetRotation).getRadians(), 0, Math.toRadians(3))) {
      speeds.vxMetersPerSecond += correctedX;
    }

    speeds.vxMetersPerSecond = xLimiter.calculate(speeds.vxMetersPerSecond);
    speeds.vyMetersPerSecond = yLimiter.calculate(speeds.vyMetersPerSecond);

    drive.drive(speeds);
  }

  public Rotation2d getTargetRotation() {
    Pose2d trapPose = getTrapPose();

    return trapPose.getRotation().plus(Rotation2d.fromDegrees(180));
  }

  public Pose2d getTrapPose() {

    boolean flipped = AllianceManager.getAlliance() == Alliance.Red;

    switch (targetStageSideSupplier.get()) {
      case LEFT:
        return !flipped ? BLUE_LEFT_TRAP : Constants.mirror(BLUE_RIGHT_TRAP);
      case RIGHT:
        return !flipped ? BLUE_RIGHT_TRAP : Constants.mirror(BLUE_LEFT_TRAP);
      default:
        return !flipped ? BLUE_CENTER_TRAP : Constants.mirror(BLUE_CENTER_TRAP);
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  private double deadband(double rawInput, double deadband) {
    if (Math.abs(rawInput) > deadband) {
      if (rawInput > 0.0) return (rawInput - deadband) / (1.0 - deadband);
      else return (rawInput + deadband) / (1.0 - deadband);
    } else return 0;
  }

  private double convertRawInput(double rawInput) {
    double deadbandInput = deadband(rawInput, deadband);
    return controllerConversion.apply(Double.valueOf(deadbandInput));
  }
}
