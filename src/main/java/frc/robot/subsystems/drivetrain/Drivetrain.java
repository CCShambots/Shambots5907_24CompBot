package frc.robot.subsystems.drivetrain;

import static frc.robot.Constants.Drivetrain.Hardware.*;
import static frc.robot.Constants.Drivetrain.Settings.*;
import static frc.robot.Constants.PhysicalConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.ShamLib.SMF.StateMachine;
import frc.robot.ShamLib.swerve.DriveCommand;
import frc.robot.ShamLib.swerve.SwerveDrive;
import frc.robot.ShamLib.swerve.SwerveSpeedLimits;
import frc.robot.ShamLib.swerve.TimestampedPoseEstimator;
import frc.robot.ShamLib.swerve.module.RealignModuleCommand;
import frc.robot.ShamLib.swerve.module.SwerveModule;
import frc.robot.util.StageSide;
import java.util.List;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.function.DoubleSupplier;
import java.util.function.IntConsumer;
import java.util.function.Supplier;

public class Drivetrain extends StateMachine<Drivetrain.State> {
  private final SwerveDrive drive;
  private boolean flipPath = false;

  private IntConsumer waypointConsumer = null;
  AtomicInteger currentWaypoint = new AtomicInteger(0);

  private final Supplier<StageSide> targetStageSideSupplier;

  private final DoubleSupplier xSupplier;
  private final DoubleSupplier ySupplier;
  private final DoubleSupplier thetaSupplier;

  public Drivetrain(
      DoubleSupplier x,
      DoubleSupplier y,
      DoubleSupplier theta,
      Supplier<StageSide> targetStageSideSupplier,
      Trigger incrementUp,
      Trigger incrementDown,
      Trigger stop) {
    super("Drivetrain", State.UNDETERMINED, State.class);

    this.xSupplier = x;
    this.ySupplier = y;
    this.thetaSupplier = theta;

    this.targetStageSideSupplier = targetStageSideSupplier;

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
            true,
            () -> flipPath,
            STATE_STD_DEVIATIONS,
            MODULE_1_INFO,
            MODULE_2_INFO,
            MODULE_3_INFO,
            MODULE_4_INFO);

    registerStateCommands(stop, incrementUp, incrementDown);
    registerTransitions();
  }

  public Pose2d getBotPose() {
    return drive.getPose();
  }

  public void setWaypointConsumer(IntConsumer waypointConsumer) {
    this.waypointConsumer = waypointConsumer;
  }

  public void syncAlliance() {
    // flip if we are on red alliance
    flipPath = Constants.alliance == DriverStation.Alliance.Red;

    // re-register face commands in case the alliance changed (they are based on the blue poses by
    // default)
    registerFaceCommands();
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
            TRAVERSE_SPEED));

    registerStateCommand(State.GROUND_INTAKE, new DriveCommand(
            drive,
            xSupplier,
            ySupplier,
            thetaSupplier,
            Constants.Controller.DEADBAND,
            Constants.Controller.DRIVE_CONVERSION,
            this,
            INTAKE_SPEED
    ));

    registerStateCommand(State.HUMAN_PLAYER_INTAKE, new DriveCommand(
            drive,
            xSupplier,
            ySupplier,
            thetaSupplier,
            Constants.Controller.DEADBAND,
            Constants.Controller.DRIVE_CONVERSION,
            this,
            HUMAN_PLAYER_PICKUP_SPEED
    ));

    registerStateCommand(
        State.AUTO_AMP,
        getPathfindCommand("AUTO_AMP", AMP_ROTATIONAL_DELAY, State.FIELD_ORIENTED_DRIVE));

    registerStateCommand(
        State.AUTO_HUMAN_PLAYER_INTAKE,
        getPathfindCommand(
            "AUTO_HUMAN_PLAYER_INTAKE",
            HUMAN_PLAYER_SCORE_ROTATIONAL_DELAY,
            State.FIELD_ORIENTED_DRIVE));

    registerStateCommand(
        State.TURN_VOLTAGE_CALC,
        drive.getTurnVoltageCalcCommand(stop, incrementUp, incrementDown, TURN_VOLTAGE_INCREMENT));
    registerStateCommand(
        State.DRIVE_VOLTAGE_CALC,
        drive.getDriveVoltageCalcCommand(
            stop, incrementUp, incrementDown, DRIVE_VOLTAGE_INCREMENT));

    registerFaceCommands();
    registerAutoClimb();
  }

  @Override
  protected void update() {
    drive.update();
  }

  private void registerTransitions() {
    addOmniTransition(State.IDLE);
    addOmniTransition(State.X_SHAPE);
    addOmniTransition(State.FIELD_ORIENTED_DRIVE);
    addOmniTransition(State.GROUND_INTAKE);
    addOmniTransition(State.HUMAN_PLAYER_INTAKE);

    addTransition(State.IDLE, State.FOLLOWING_AUTONOMOUS_TRAJECTORY);

    addTransition(State.IDLE, State.TURN_VOLTAGE_CALC);
    addTransition(State.IDLE, State.DRIVE_VOLTAGE_CALC);

    addOmniTransition(State.AUTO_AMP);
    addOmniTransition(State.AUTO_CLIMB);
    addOmniTransition(State.AUTO_HUMAN_PLAYER_INTAKE);

    addOmniTransition(State.FACE_RIGHT_TRAP);
    addOmniTransition(State.FACE_LEFT_TRAP);
    addOmniTransition(State.FACE_CENTER_TRAP);
    addOmniTransition(State.FACE_AMP);
    addOmniTransition(State.FACE_SPEAKER);
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
        State.FACE_CENTER_TRAP,
        getFacePointCommand(
            flipPath ? Constants.mirror(BLUE_CENTER_TRAP) : BLUE_CENTER_TRAP, TRAP_SPEED));

    registerStateCommand(
        State.FACE_LEFT_TRAP,
        getFacePointCommand(
            flipPath ? Constants.mirror(BLUE_LEFT_TRAP) : BLUE_LEFT_TRAP, TRAP_SPEED));

    registerStateCommand(
        State.FACE_RIGHT_TRAP,
        getFacePointCommand(
            flipPath ? Constants.mirror(BLUE_RIGHT_TRAP) : BLUE_RIGHT_TRAP, TRAP_SPEED));
  }

  private Command getFacePointCommand(Pose2d pose, SwerveSpeedLimits limits) {
    FacePointCommand facePointCommand =
        new FacePointCommand(
            drive,
            HOLD_ANGLE_GAINS,
            pose,
            drive::getPose,
            xSupplier,
            ySupplier,
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

  private Command notifyWaypointCommand() {
    return new InstantCommand(
        () -> {
          if (waypointConsumer != null) waypointConsumer.accept(currentWaypoint.getAndIncrement());
        });
  }

  public void addVisionMeasurements(
      List<TimestampedPoseEstimator.TimestampedVisionUpdate> measurement) {
    drive.addTimestampedVisionMeasurements(measurement);
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
    FOLLOWING_AUTONOMOUS_TRAJECTORY,
    FACE_SPEAKER,
    FACE_AMP,
    FACE_CENTER_TRAP,
    FACE_LEFT_TRAP,
    FACE_RIGHT_TRAP,
    AUTO_AMP,
    AUTO_CLIMB,
    AUTO_HUMAN_PLAYER_INTAKE,
    TURN_VOLTAGE_CALC,
    DRIVE_VOLTAGE_CALC,
    GROUND_INTAKE,
    HUMAN_PLAYER_INTAKE,

    // flags for non-autonomous operations
    PATHFINDING,
    AT_ANGLE
  }
}
