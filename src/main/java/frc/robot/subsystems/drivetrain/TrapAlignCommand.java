package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.ShamLib.PIDGains;
import frc.robot.ShamLib.swerve.SwerveDrive;

public class TrapAlignCommand extends Command {

  private final SwerveDrive drive;
  private final Drivetrain dt;

  private final PIDController xController;
  private final PIDController yController;
  private final PIDController thetaController;

  private boolean finished = false;

  public TrapAlignCommand(
      SwerveDrive drive, Drivetrain dt, PIDGains translationController, PIDGains thetaGains) {
    this.drive = drive;
    this.dt = dt;

    this.xController = translationController.applyToController();
    this.yController = translationController.applyToController();

    this.thetaController = thetaGains.applyToController();
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void initialize() {
    finished = false;
  }

  @Override
  public void execute() {
    Pose2d targetedTrap = dt.getCurrentTrapPose();

    Pose2d botPoseTarget =
        targetedTrap.transformBy(
            new Transform2d(
                Constants.Drivetrain.Settings.TRAP_OFFSET, Rotation2d.fromDegrees(180)));

    Pose2d botPose = drive.getPose();

    ChassisSpeeds fieldSpeeds =
        new ChassisSpeeds(
            xController.calculate(botPose.getX(), botPoseTarget.getX()),
            yController.calculate(botPose.getY(), botPoseTarget.getY()),
            thetaController.calculate(
                botPose.getRotation().getRadians(), botPoseTarget.getRotation().getRadians()));

    ChassisSpeeds chassisRelativeField =
        ChassisSpeeds.fromFieldRelativeSpeeds(fieldSpeeds, drive.getCurrentAngle());

    // Logger.recordOutput("Drivetrain/Bot Target", botPoseTarget);
    drive.drive(chassisRelativeField);

    finished =
        botPose.getTranslation().getDistance(botPoseTarget.getTranslation())
                < Units.inchesToMeters(0.5)
            && Math.abs(
                    botPose.getRotation().getDegrees() - botPoseTarget.getRotation().getDegrees())
                < 2;
  }

  @Override
  public boolean isFinished() {
    return finished;
  }

  @Override
  public void end(boolean interrupted) {
    drive.stopModules();
    // dt.requestTransition(Drivetrain.State.X_SHAPE);
  }
}
