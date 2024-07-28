package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.ShamLib.AllianceManager;
import frc.robot.util.StageSide;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Supplier;

public class AutoStageSelectCommand extends Command {

  private Consumer<StageSide> setStageSide;
  private Supplier<Pose2d> poseSupplier;
  private BooleanSupplier shouldEnd;

  public AutoStageSelectCommand(
      Consumer<StageSide> setStageSide, Supplier<Pose2d> poseSupplier, BooleanSupplier shouldEnd) {
    this.setStageSide = setStageSide;
    this.poseSupplier = poseSupplier;
    this.shouldEnd = shouldEnd;
  }

  @Override
  public void execute() {
    Translation2d currentTranslation = poseSupplier.get().getTranslation();

    double leftDistance = currentTranslation.getDistance(getTrapPos(StageSide.LEFT));
    double rightDistance = currentTranslation.getDistance(getTrapPos(StageSide.RIGHT));
    double centerDistance = currentTranslation.getDistance(getTrapPos(StageSide.CENTER));

    if (leftDistance < rightDistance && leftDistance < centerDistance) {
      setStageSide.accept(StageSide.LEFT);
    } else if (rightDistance < leftDistance && rightDistance < centerDistance) {
      setStageSide.accept(StageSide.RIGHT);
    } else {
      setStageSide.accept(StageSide.CENTER);
    }
  }

  private Translation2d getTrapPos(StageSide side) {
    switch (side) {
      case LEFT:
        return handleFlip(Constants.PhysicalConstants.BLUE_LEFT_TRAP).getTranslation();
      case RIGHT:
        return handleFlip(Constants.PhysicalConstants.BLUE_RIGHT_TRAP).getTranslation();
      case CENTER:
        return handleFlip(Constants.PhysicalConstants.BLUE_CENTER_TRAP).getTranslation();
      default:
        return handleFlip(Constants.PhysicalConstants.BLUE_CENTER_TRAP).getTranslation();
    }
  }

  private Pose2d handleFlip(Pose2d translation) {
    if (AllianceManager.getAlliance() == Alliance.Red) return Constants.mirror(translation);
    else return translation;
  }

  @Override
  public boolean isFinished() {
    return shouldEnd.getAsBoolean();
  }
}
