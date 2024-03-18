package frc.robot.subsystems.drivetrain;

import static frc.robot.Constants.Drivetrain.Hardware.*;
import static frc.robot.Constants.Drivetrain.Settings.*;
import static frc.robot.Constants.PhysicalConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.ShamLib.AllianceManager;
import frc.robot.ShamLib.SMF.StateMachine;
import frc.robot.ShamLib.WhileDisabledInstantCommand;
import frc.robot.ShamLib.swerve.DriveCommand;
import frc.robot.ShamLib.swerve.SwerveDrive;
import frc.robot.ShamLib.swerve.SwerveSpeedLimits;
import frc.robot.ShamLib.swerve.TimestampedPoseEstimator;
import frc.robot.ShamLib.swerve.module.RealignModuleCommand;
import frc.robot.ShamLib.swerve.module.SwerveModule;
import frc.robot.subsystems.vision.Vision.RingVisionUpdate;
import frc.robot.util.StageSide;
import java.util.List;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.IntConsumer;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Drivetrain extends StateMachine<Drivetrain.State> {
  private final SwerveDrive drive;
  private boolean flipPath = false;

  private IntConsumer waypointConsumer = null;
  AtomicInteger currentWaypoint = new AtomicInteger(0);

  private final Supplier<StageSide> targetStageSideSupplier;

  private final DoubleSupplier xSupplier;
  private final DoubleSupplier ySupplier;
  private final DoubleSupplier thetaSupplier;

  private RingVisionUpdate ringVisionUpdate;

  private final BooleanSupplier intakeProxTripped;
  private final BooleanSupplier indexerReceivedRing;

  public Drivetrain(
      DoubleSupplier x,
      DoubleSupplier y,
      DoubleSupplier theta,
      Supplier<StageSide> targetStageSideSupplier,
      Trigger incrementUp,
      Trigger incrementDown,
      Trigger stop,
      BooleanSupplier intakeProxTripped,
      BooleanSupplier indexerReceivedRing) {
    super("Drivetrain", State.UNDETERMINED, State.class);

    this.xSupplier = x;
    this.ySupplier = y;
    this.thetaSupplier = theta;

    this.targetStageSideSupplier = targetStageSideSupplier;

    this.intakeProxTripped = intakeProxTripped;
    this.indexerReceivedRing = indexerReceivedRing;

    NamedCommands.registerCommand("notifyNextWaypoint", notifyWaypointCommand());
    NamedCommands.registerCommand(
        "clearPathfindingFlag", new InstantCommand(() -> clearFlag(State.PATHFINDING)));

    drive =
        new SwerveDrive(
            Constants.currentBuildMode,
            GYRO_ID,
            MODULE_DRIVE_GAINS,
            MODULE_TURN_GAINS,
            MAX_CHASSIS_SPEED,
            MAX_CHASSIS_ACCELERATION,
            MAX_CHASSIS_ROTATIONAL_SPEED,
            MAX_CHASSIS_ROTATIONAL_ACCELERATION,
            MAX_MODULE_TURN_SPEED,
            MAX_MODULE_TURN_ACCELERATION,
            AUTO_THETA_GAINS,
            AUTO_TRANSLATION_GAINS,
            false,
            MODULE_CAN_BUS,
            GYRO_CAN_BUS,
            CURRENT_LIMITS_CONFIGS,
            this,
            false,
            () -> flipPath,
            STATE_STD_DEVIATIONS,
            Constants.LOOP_PERIOD,
            MODULE_1_INFO,
            MODULE_2_INFO,
            MODULE_3_INFO,
            MODULE_4_INFO);

    registerStateCommands(stop, incrementUp, incrementDown);
    registerTransitions();

    // Make sure the alliaince flipping gets passed on to the drivetrain
    AllianceManager.addAllianceChangeHook(this::syncAlliance);

    PPHolonomicDriveController.setRotationTargetOverride(null);
  }

  public Pose2d getBotPose() {
    return drive.getPose();
  }

  public void setWaypointConsumer(IntConsumer waypointConsumer) {
    this.waypointConsumer = waypointConsumer;
  }

  public void syncAlliance() {
    // flip if we are on red alliance
    flipPath = AllianceManager.getAlliance() == DriverStation.Alliance.Red;

    // re-register face commands in case the alliance changed (they are based on the blue poses by
    // default)
    registerFaceCommands();

    syncTargetStageSide();

    System.out.println(AllianceManager.getAlliance());
    System.out.println("Path Flipping:" + flipPath);
  }

  public Pose2d getCurrentTrapPose() {
    switch (targetStageSideSupplier.get()) {
      case LEFT:
        return !flipPath ? BLUE_LEFT_TRAP : Constants.mirror(BLUE_RIGHT_TRAP);
      case RIGHT:
        return !flipPath ? BLUE_RIGHT_TRAP : Constants.mirror(BLUE_LEFT_TRAP);
      default:
        return !flipPath ? BLUE_CENTER_TRAP : Constants.mirror(BLUE_CENTER_TRAP);
    }
  }

  public void syncTargetStageSide() {
    // registerStateCommand(
    // State.TRAP, getPathFindCommand(getTrapTarget(), TRAP_ROTATIONAL_DELAY, State.X_SHAPE));
  }

  public void setAutonomousCommand(Command command) {
    registerStateCommand(State.FOLLOWING_AUTONOMOUS_TRAJECTORY, command);
  }

  private void registerStateCommands(Trigger stop, Trigger incrementUp, Trigger incrementDown) {
    registerStateCommand(State.X_SHAPE, () -> drive.setModuleStates(X_SHAPE));

    registerStateCommand(State.IDLE, drive::stopModules);

    registerStateCommand(
        State.FIELD_ORIENTED_DRIVE,
        new DriveCommand(
                drive,
                xSupplier,
                ySupplier,
                thetaSupplier,
                Constants.Controller.DEADBAND,
                Constants.Controller.DRIVE_CONVERSION,
                this,
                TRAVERSE_SPEED)
            .alongWith(new InstantCommand(() -> setFlag(State.AT_ANGLE))));

    registerStateCommand(
        State.GROUND_INTAKE,
        new DriveCommand(
            drive,
            xSupplier,
            ySupplier,
            thetaSupplier,
            Constants.Controller.DEADBAND,
            Constants.Controller.DRIVE_CONVERSION,
            this,
            INTAKE_SPEED));

    registerStateCommand(
        State.AUTO_GROUND_INTAKE,
        new SequentialCommandGroup(
            setFlagCommand(State.AUTO_INTAKING),
            new AutoIntakeCommand(
                drive,
                () -> ringVisionUpdate,
                INTAKE_SPEED,
                AUTO_THETA_GAINS,
                intakeProxTripped,
                LOST_RING_TARGET_TIMEOUT,
                indexerReceivedRing),
            clearFlagCommand(State.AUTO_INTAKING)));

    registerStateCommand(
        State.HUMAN_PLAYER_INTAKE,
        new DriveCommand(
            drive,
            xSupplier,
            ySupplier,
            thetaSupplier,
            Constants.Controller.DEADBAND,
            Constants.Controller.DRIVE_CONVERSION,
            this,
            HUMAN_PLAYER_PICKUP_SPEED));

    registerStateCommand(
        State.CHAIN_ORIENTED_DRIVE,
        new ChainRelativeDriveCommand(
            drive,
            AUTO_THETA_GAINS,
            targetStageSideSupplier,
            xSupplier,
            ySupplier,
            Constants.Controller.DEADBAND,
            Constants.Controller.DRIVE_CONVERSION,
            AMP_SPEED));

    registerStateCommand(
        State.TURN_VOLTAGE_CALC,
        drive.getTurnVoltageCalcCommand(stop, incrementUp, incrementDown, TURN_VOLTAGE_INCREMENT));
    registerStateCommand(
        State.DRIVE_VOLTAGE_CALC,
        drive.getDriveVoltageCalcCommand(
            stop, incrementUp, incrementDown, DRIVE_VOLTAGE_INCREMENT));

    registerStateCommand(
        State.TRAP, new TrapAlignCommand(drive, this, TRAP_TRANSLATION_GAINS, TRAP_THETA_GAINS));

    registerFaceCommands();
  }

  @Override
  protected void update() {
    drive.update();

    Logger.recordOutput(
        "Drivetrain/trap-distance",
        getCurrentTrapPose().getTranslation().getDistance(drive.getPose().getTranslation()));
    Logger.recordOutput(
        "Drivetrain/trap-distance-x",
        Math.abs(
            getCurrentTrapPose().getTranslation().getX()
                - drive.getPose().getTranslation().getX()));
    Logger.recordOutput(
        "Drivetrain/trap-distance-y",
        Math.abs(
            getCurrentTrapPose().getTranslation().getY()
                - drive.getPose().getTranslation().getY()));
    Logger.recordOutput("Drivetrain/trap-pose", getCurrentTrapPose());
  }

  private void registerTransitions() {
    addOmniTransition(State.IDLE, () -> PPHolonomicDriveController.setRotationTargetOverride(null));
    addOmniTransition(State.X_SHAPE);
    addOmniTransition(State.FIELD_ORIENTED_DRIVE);
    addOmniTransition(State.GROUND_INTAKE);
    addOmniTransition(State.AUTO_GROUND_INTAKE);
    addOmniTransition(State.HUMAN_PLAYER_INTAKE);

    addTransition(State.IDLE, State.FOLLOWING_AUTONOMOUS_TRAJECTORY);
    addTransition(
        State.IDLE, State.FOLLOWING_AUTONOMOUS_TRAJECTORY_AIMING, this::ppRotationOverride);
    addTransition(
        State.FOLLOWING_AUTONOMOUS_TRAJECTORY_AIMING,
        State.FOLLOWING_AUTONOMOUS_TRAJECTORY,
        () -> PPHolonomicDriveController.setRotationTargetOverride(null));
    addTransition(
        State.FOLLOWING_AUTONOMOUS_TRAJECTORY,
        State.FOLLOWING_AUTONOMOUS_TRAJECTORY_AIMING,
        this::ppRotationOverride);

    addTransition(State.IDLE, State.TURN_VOLTAGE_CALC);
    addTransition(State.IDLE, State.DRIVE_VOLTAGE_CALC);

    addOmniTransition(State.AUTO_AMP);
    addOmniTransition(State.AUTO_CLIMB);
    addOmniTransition(State.AUTO_HUMAN_PLAYER_INTAKE);

    addOmniTransition(State.TRAP);
    addOmniTransition(State.FACE_AMP);
    addOmniTransition(State.FACE_SPEAKER);
    addOmniTransition(State.FACE_SPEAKER_AUTO);

    addOmniTransition(State.LOB);

    addOmniTransition(State.CHAIN_ORIENTED_DRIVE);
  }

  private void ppRotationOverride() {
    PPHolonomicDriveController.setRotationTargetOverride(
        () -> Optional.of(getTargetRotationToSpeaker()));
  }

  private Rotation2d getTargetRotationToSpeaker() {
    return Constants.rotationBetween(
            drive.getPose(),
            flipPath
                ? Constants.PhysicalConstants.BLUE_SPEAKER
                : Constants.mirror(Constants.PhysicalConstants.BLUE_SPEAKER))
        .minus(Constants.Drivetrain.Settings.SHOT_OFFSET);
  }

  private void registerFaceCommands() {
    registerStateCommand(
        State.FACE_AMP,
        getFacePointCommand(flipPath ? Constants.mirror(BLUE_AMP) : BLUE_AMP, AMP_SPEED));

    registerStateCommand(
        State.FACE_SPEAKER,
        getFacePointCommand(
            flipPath ? Constants.mirror(BLUE_SPEAKER) : BLUE_SPEAKER, SPEAKER_SPEED));

    registerStateCommand(
        State.LOB,
        getFacePointCommand(
            flipPath ? Constants.mirror(BLUE_LOB_CORNER) : BLUE_LOB_CORNER, LOB_SPEED));

    registerStateCommand(
        State.FACE_SPEAKER_AUTO,
        getFacePointCommand(
            flipPath ? Constants.mirror(BLUE_SPEAKER) : BLUE_SPEAKER, SPEAKER_SPEED, true));
  }

  private void registerPathFollowStateCommands() {
    registerStateCommand(
        State.AUTO_AMP,
        getPathfindCommand("AUTO_AMP", AMP_ROTATIONAL_DELAY, State.FIELD_ORIENTED_DRIVE));

    registerStateCommand(
        State.AUTO_HUMAN_PLAYER_INTAKE,
        getPathfindCommand(
            "AUTO_HUMAN_PLAYER_INTAKE",
            HUMAN_PLAYER_SCORE_ROTATIONAL_DELAY,
            State.FIELD_ORIENTED_DRIVE));

    registerAutoClimb();
  }

  private Command getFacePointCommand(Pose2d pose, SwerveSpeedLimits limits) {
    return getFacePointCommand(pose, limits, false);
  }

  private Command getFacePointCommand(Pose2d pose, SwerveSpeedLimits limits, boolean usedInAuto) {
    FacePointCommand facePointCommand =
        new FacePointCommand(
            drive,
            AUTO_THETA_GAINS,
            pose,
            drive::getPose,
            !usedInAuto ? xSupplier : () -> 0,
            !usedInAuto ? ySupplier : () -> 0,
            Constants.Controller.DEADBAND,
            Constants.Controller.DRIVE_CONVERSION,
            this,
            limits);

    return new ParallelCommandGroup(
        facePointCommand,
        new RunCommand(
            () -> {
              if (facePointCommand.atAngle(FACE_ANGLE_TOLERANCE)) {
                setFlag(State.AT_ANGLE);
              } else {
                clearFlag(State.AT_ANGLE);
              }
            }));
  }

  private Command getPathfindCommand(String nextPath, double rotationalDelay, State endState) {
    return new SequentialCommandGroup(
        new InstantCommand(() -> setFlag(State.PATHFINDING)),
        AutoBuilder.pathfindThenFollowPath(
            PathPlannerPath.fromPathFile(nextPath), getPathfindingConstraints(), rotationalDelay),
        transitionCommand(endState, false));
  }

  private Command getPathFindCommand(Pose2d targetPose, double rotationalDelay, State endState) {
    return new SequentialCommandGroup(
        new InstantCommand(() -> setFlag(State.PATHFINDING)),
        AutoBuilder.pathfindToPose(targetPose, getPathfindingConstraints(), rotationalDelay),
        transitionCommand(endState, false));
  }

  private Pose2d getTrapTarget() {
    Pose2d trapPose;

    switch (targetStageSideSupplier.get()) {
      case LEFT:
        trapPose = !flipPath ? BLUE_LEFT_TRAP : BLUE_RIGHT_TRAP;
        break;
      case RIGHT:
        trapPose = !flipPath ? BLUE_RIGHT_TRAP : BLUE_LEFT_TRAP;
        break;
      default:
        trapPose = BLUE_CENTER_TRAP;
        break;
    }

    Pose2d targetPose =
        trapPose.transformBy(
            new Transform2d(
                new Pose2d(), new Pose2d(TRAP_SHOT_DISTANCE, 0, Rotation2d.fromDegrees(180))));

    System.out.println("FLIPPED: " + flipPath);
    System.out.println("alliance at time of run: " + DriverStation.getAlliance());
    targetPose = flipPath ? Constants.mirror(targetPose) : targetPose;

    return targetPose;
  }

  private PathConstraints getPathfindingConstraints() {
    return new PathConstraints(
        PATH_FIND_SPEED.getMaxSpeed(),
        PATH_FIND_SPEED.getMaxAcceleration(),
        PATH_FIND_SPEED.getMaxRotationalSpeed(),
        PATH_FIND_SPEED.getMaxRotationalAcceleration());
  }

  public void registerAutoClimb() {
    registerStateCommand(
        State.AUTO_CLIMB,
        getPathfindCommand(
            "AUTO_CLIMB_" + targetStageSideSupplier.get(),
            CLIMB_ROTATION_DELAY,
            State.FIELD_ORIENTED_DRIVE));
  }

  public Command resetGyro() {
    return new WhileDisabledInstantCommand(
        () -> {
          drive.resetGyro(Rotation2d.fromDegrees(180));
        });
  }

  public void resetFieldOriented() {
    Rotation2d newAngle = drive.getPose().getRotation();

    // Flip by 180 if we're on red alliance
    // if (flipPath)
    newAngle = newAngle.plus(new Rotation2d(Math.PI));

    drive.resetFieldOrientedRotationOffset(newAngle);
  }

  private Command notifyWaypointCommand() {
    return new InstantCommand(
        () -> {
          if (waypointConsumer != null) waypointConsumer.accept(currentWaypoint.getAndIncrement());
        });
  }

  public void configurePathplanner() {
    drive.configurePathplanner();

    registerPathFollowStateCommands();
  }

  public void addVisionMeasurements(
      List<TimestampedPoseEstimator.TimestampedVisionUpdate> measurement) {
    drive.addTimestampedVisionMeasurements(measurement);
  }

  public void recordRingMeasurement(RingVisionUpdate visionUpdate) {
    this.ringVisionUpdate = visionUpdate;
  }

  public Pose2d getCurrentLobPose() {
    return flipPath ? BLUE_LOB_CORNER : Constants.mirror(BLUE_LOB_CORNER);
  }

  public void registerMisalignedSwerveTriggers(EventLoop loop) {
    for (SwerveModule module : drive.getModules()) {
      loop.bind(
          () -> {
            if (module.isModuleMisaligned() && !isEnabled()) {
              new RealignModuleCommand(module).schedule();
            }
          });
    }
  }

  @Override
  protected void determineSelf() {
    setState(State.IDLE);
  }

  public enum State {
    UNDETERMINED,
    IDLE,
    X_SHAPE,
    FIELD_ORIENTED_DRIVE,
    CHAIN_ORIENTED_DRIVE,
    FOLLOWING_AUTONOMOUS_TRAJECTORY,
    FOLLOWING_AUTONOMOUS_TRAJECTORY_AIMING,
    FACE_SPEAKER,
    FACE_SPEAKER_AUTO,
    FACE_AMP,
    TRAP,
    AUTO_AMP,
    AUTO_CLIMB,
    AUTO_GROUND_INTAKE,
    AUTO_HUMAN_PLAYER_INTAKE,
    TURN_VOLTAGE_CALC,
    DRIVE_VOLTAGE_CALC,
    GROUND_INTAKE,
    HUMAN_PLAYER_INTAKE,
    LOB,

    // flags for non-autonomous operations
    PATHFINDING,
    AT_ANGLE,
    AT_TRAP_POSE,
    AUTO_INTAKING
  }
}
