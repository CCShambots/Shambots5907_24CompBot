package frc.robot.subsystems.drivetrain;

import static frc.robot.Constants.PhysicalConstants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.ShamLib.AllianceManager;
import frc.robot.ShamLib.PIDGains;
import frc.robot.ShamLib.swerve.SwerveDrive;
import frc.robot.ShamLib.swerve.SwerveSpeedLimits;
import frc.robot.util.StageSide;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import java.util.function.UnaryOperator;

public class ChainRelativeDriveCommand extends Command {

  private final SwerveDrive drive;
  private final Supplier<StageSide> targetStageSideSupplier;
  private final DoubleSupplier xSupplier;
  private final DoubleSupplier ySupplier;
  private final SwerveSpeedLimits speedLimits;

  private final PIDController thetaController;

  private final SlewRateLimiter xLimiter;
  private final SlewRateLimiter yLimiter;
  private final SlewRateLimiter thetaLimiter;

  private final double deadband;
  private final UnaryOperator<Double> controllerConversion;

  public ChainRelativeDriveCommand(
      SwerveDrive drive,
      PIDGains thetaGains,
      Supplier<StageSide> targetStageSideSupplier,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      double deadband,
      UnaryOperator<Double> controllerConversion,
      SwerveSpeedLimits speedLimits) {
    this.drive = drive;
    this.targetStageSideSupplier = targetStageSideSupplier;
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    this.speedLimits = speedLimits;

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
    Rotation2d currentRotation = drive.getPose().getRotation();
    Rotation2d targetRotation = getTargetRotation();

    double correctedX = convertRawInput(xSupplier.getAsDouble()) * speedLimits.getMaxSpeed();
    double correctedY = convertRawInput(ySupplier.getAsDouble()) * speedLimits.getMaxSpeed();
    if (targetStageSideSupplier.get() == StageSide.CENTER) correctedY *= -1;
    double correctedRot =
        thetaController.calculate(currentRotation.getRadians(), targetRotation.getRadians());

    ChassisSpeeds speeds = new ChassisSpeeds(correctedX, correctedY, correctedRot);

    speeds.vxMetersPerSecond = xLimiter.calculate(speeds.vxMetersPerSecond);
    speeds.vyMetersPerSecond = yLimiter.calculate(speeds.vyMetersPerSecond);
    // speeds.omegaRadiansPerSecond = thetaLimiter.calculate(speeds.omegaRadiansPerSecond);

    drive.drive(speeds);
  }

  public Rotation2d getTargetRotation() {
    Pose2d trapPose;

    boolean flipped = AllianceManager.getAlliance() == Alliance.Red;

    switch (targetStageSideSupplier.get()) {
      case LEFT:
        trapPose = !flipped ? BLUE_LEFT_TRAP : Constants.mirror(BLUE_RIGHT_TRAP);
        break;
      case RIGHT:
        trapPose = !flipped ? BLUE_RIGHT_TRAP : Constants.mirror(BLUE_LEFT_TRAP);
        break;
      default:
        trapPose = !flipped ? BLUE_CENTER_TRAP : Constants.mirror(BLUE_CENTER_TRAP);
        break;
    }

    return trapPose.getRotation().plus(Rotation2d.fromDegrees(180));
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
