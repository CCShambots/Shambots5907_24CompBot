// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.ShamLib.AllianceManager;
import frc.robot.ShamLib.SMF.StateMachine;
import frc.robot.ShamLib.ShamLibConstants;
import frc.robot.ShamLib.ShamLibConstants.BuildMode;
import frc.robot.ShamLib.WhileDisabledInstantCommand;
import frc.robot.commands.DetermineRingStatusCommand;
import frc.robot.controllers.ControllerBindings;
import frc.robot.controllers.RealControllerBindings;
import frc.robot.controllers.SimControllerBindings;
import frc.robot.subsystems.climbers.ClimberIO;
import frc.robot.subsystems.climbers.ClimberIOReal;
import frc.robot.subsystems.climbers.ClimberIOSim;
import frc.robot.subsystems.climbers.Climbers;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerIO;
import frc.robot.subsystems.indexer.IndexerIOReal;
import frc.robot.subsystems.indexer.IndexerIOSim;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOReal;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.lights.Lights;
import frc.robot.subsystems.lights.LightsIO;
import frc.robot.subsystems.lights.LightsIOReal;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.arm.ArmIO;
import frc.robot.subsystems.shooter.arm.ArmIOReal;
import frc.robot.subsystems.shooter.arm.ArmIOSim;
import frc.robot.subsystems.shooter.flywheel.FlywheelIO;
import frc.robot.subsystems.shooter.flywheel.FlywheelIOReal;
import frc.robot.subsystems.shooter.flywheel.FlywheelIOSim;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.StageSide;
import java.util.Map;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer extends StateMachine<RobotContainer.State> {
  private final Intake intake;
  private final Shooter shooter;
  private final Indexer indexer;
  private final Vision vision;
  private final Climbers climbers;
  private final Drivetrain drivetrain;
  private final Lights lights;

  // Controller bindings object that will be created to handle both real control and sim inputs
  private final ControllerBindings controllerBindings;

  private final LoggedDashboardChooser<Command> autoChooser;

  @AutoLogOutput private StageSide targetStageSide = StageSide.CENTER;

  private Drivetrain.State prevDTState = Drivetrain.State.FIELD_ORIENTED_DRIVE;

  private boolean poseWorking = true;
  private boolean autoIntakeWorking = true;

  public RobotContainer(EventLoop checkModulesLoop) {
    super("RobotContainer", State.UNDETERMINED, State.class);

    if (Constants.currentBuildMode == ShamLibConstants.BuildMode.SIM) {
      controllerBindings = new SimControllerBindings();
    } else {
      controllerBindings = new RealControllerBindings();
    }

    intake =
        new Intake(
            getIntakeIO(controllerBindings.simProx1()),
            tuningIncrement(),
            tuningDecrement(),
            tuningStop());

    indexer =
        new Indexer(
            getIndexerIO(
                controllerBindings.simProx2(),
                controllerBindings.simProx3(),
                controllerBindings.simProx4()),
            tuningIncrement(),
            tuningDecrement(),
            tuningStop());

    Map<String, Pose3d> photonMap =
        Constants.currentBuildMode == BuildMode.SIM
            ? Map.of()
            : Map.of(
                "pv_instance_1",
                Constants.Vision.Hardware.RIGHT_CAM_POSE,
                "pv_instance_2",
                Constants.Vision.Hardware.LEFT_CAM_POSE);

    vision = new Vision("limelight", photonMap);

    drivetrain =
        new Drivetrain(
            controllerBindings::getDriveXValue,
            controllerBindings::getDriveYValue,
            controllerBindings::getDriveTurnValue,
            () -> targetStageSide,
            tuningIncrement(),
            tuningDecrement(),
            tuningStop(),
            () -> intake.isFlag(Intake.State.PROX_TRIPPED),
            () -> indexer.getState() == Indexer.State.INDEXING);

    drivetrain.registerMisalignedSwerveTriggers(checkModulesLoop);

    vision.addRingVisionUpdateConsumers(drivetrain::recordRingMeasurement);

    vision.addVisionUpdateConsumers(drivetrain::addVisionMeasurements);

    climbers =
        new Climbers(
            getLeftClimberIO(),
            getRightClimberIO(),
            tuningIncrement(),
            tuningDecrement(),
            tuningStop());

    lights = new Lights(getLightsIO());

    shooter =
        new Shooter(
            getArmIO(),
            getFlywheelIO(),
            () -> drivetrain.getBotPose().getTranslation(),
            () -> targetStageSide,
            tuningIncrement(),
            tuningDecrement(),
            tuningStop());

    addChildSubsystem(drivetrain);
    addChildSubsystem(vision);
    addChildSubsystem(intake);
    addChildSubsystem(shooter);
    addChildSubsystem(indexer);
    addChildSubsystem(climbers);

    lights.enable();

    registerStateCommands();
    registerTransitions();

    configureBindings();

    registerNamedCommands();

    // Important to instatiate after drivetrain consructor is called so that auto builder is
    // configured
    autoChooser =
        new LoggedDashboardChooser<>("Logged Autonomous Chooser", AutoBuilder.buildAutoChooser());

    initializeDriveTab();
  }

  private void registerNamedCommands() {
    // TODO: set up if we ever use automatic climbing
    NamedCommands.registerCommand("raiseClimber", new InstantCommand());

    NamedCommands.registerCommand(
        "intake",
        new ParallelCommandGroup(
            intake.transitionCommand(Intake.State.INTAKE, false),
            indexer.transitionCommand(Indexer.State.EXPECT_RING_BACK)));
    NamedCommands.registerCommand("stopIntake", intake.transitionCommand(Intake.State.IDLE, false));

    NamedCommands.registerCommand(
        "shoot",
        new SequentialCommandGroup(
            indexer.waitForState(Indexer.State.HOLDING_RING).withTimeout(3),
            indexer.transitionCommand(Indexer.State.FEED_TO_SHOOTER, false)));

    NamedCommands.registerCommand(
        "fireSequence",
        new SequentialCommandGroup(
            indexer.waitForState(Indexer.State.HOLDING_RING).withTimeout(3),
            intake.transitionCommand(Intake.State.IDLE, false),
            indexer.transitionCommand(Indexer.State.FEED_TO_SHOOTER, false),
            new WaitCommand(0.25),
            new ParallelCommandGroup(
                intake.transitionCommand(Intake.State.INTAKE, false),
                new InstantCommand(
                    () -> {
                      new SequentialCommandGroup(
                              indexer.waitForState(Indexer.State.IDLE),
                              indexer.transitionCommand(Indexer.State.EXPECT_RING_BACK))
                          .schedule();
                    }))));

    drivetrain.configurePathplanner();
  }

  private void registerStateCommands() {
    registerStateCommand(
        State.SOFT_E_STOP,
        new ParallelCommandGroup(
            drivetrain.transitionCommand(Drivetrain.State.IDLE),
            climbers.transitionCommand(Climbers.State.SOFT_E_STOP),
            intake.transitionCommand(Intake.State.IDLE),
            shooter.transitionCommand(Shooter.State.SOFT_E_STOP),
            indexer.transitionCommand(Indexer.State.IDLE),
            lights.transitionCommand(Lights.State.ERROR)));

    registerStateCommand(
        State.TRAVERSING,
        new ParallelCommandGroup(
            drivetrain.transitionCommand(Drivetrain.State.FIELD_ORIENTED_DRIVE),
            climbers.transitionCommand(Climbers.State.FREE_RETRACT),
            intake.transitionCommand(Intake.State.IDLE),
            new ConditionalCommand(
                shooter.partialFlywheelSpinup(), new InstantCommand(), () -> indexer.ringPresent()),
            new DetermineRingStatusCommand(shooter, indexer, lights)));

    registerStateCommand(
        State.SPEAKER_SCORE,
        new SequentialCommandGroup(
            // face speaker and idle intake
            drivetrain.transitionCommand(Drivetrain.State.FACE_SPEAKER),
            intake.transitionCommand(Intake.State.IDLE),
            // figure out ring issues (if there are any)
            new DetermineRingStatusCommand(shooter, indexer, lights),
            // have shooter start to track
            shooter.transitionCommand(Shooter.State.SPEAKER_AA),
            // lights show green on ready and feed ring on press, transition to traversing after
            // ring is fed
            new ParallelCommandGroup(
                lightsOnReadyCommand(Lights.State.TARGETING), feedOnPress(State.TRAVERSING))));

    registerStateCommand(
        State.GROUND_INTAKE,
        new SequentialCommandGroup(
            drivetrain.transitionCommand(Drivetrain.State.GROUND_INTAKE),
            shooter.transitionCommand(Shooter.State.STOW),
            indexer.transitionCommand(Indexer.State.EXPECT_RING_BACK),
            intake.transitionCommand(Intake.State.INTAKE),
            indexer.waitForState(Indexer.State.INDEXING),
            transitionCommand(State.TRAVERSING)));

    registerStateCommand(
        State.BASE_SHOT,
        new SequentialCommandGroup(
            drivetrain.transitionCommand(Drivetrain.State.FIELD_ORIENTED_DRIVE),
            intake.transitionCommand(Intake.State.IDLE),
            new DetermineRingStatusCommand(shooter, indexer, lights),
            shooter.transitionCommand(Shooter.State.BASE_SHOT),
            new ParallelCommandGroup(
                lightsOnReadyCommand(Lights.State.TARGETING), feedOnPress(State.TRAVERSING))));

    registerStateCommand(
        State.HUMAN_PLAYER_INTAKE,
        new ParallelCommandGroup(
                drivetrain.transitionCommand(Drivetrain.State.HUMAN_PLAYER_INTAKE),
                intake.transitionCommand(Intake.State.IDLE),
                lights.transitionCommand(Lights.State.INTAKE),
                shooter.transitionCommand(Shooter.State.CHUTE_INTAKE),
                indexer.transitionCommand(Indexer.State.EXPECT_RING_FRONT))
            .andThen(
                new WaitUntilCommand(
                    () ->
                        indexer.getState() == Indexer.State.HOLDING_RING
                            || indexer.getState() == Indexer.State.LOST_RING))
            .andThen(transitionCommand(State.TRAVERSING)));

    registerStateCommand(
        State.AUTO_HP_INTAKE,
        new ParallelCommandGroup(
                drivetrain.transitionCommand(Drivetrain.State.AUTO_HUMAN_PLAYER_INTAKE),
                intake.transitionCommand(Intake.State.IDLE),
                lights.transitionCommand(Lights.State.INTAKE),
                shooter.transitionCommand(Shooter.State.CHUTE_INTAKE),
                indexer.transitionCommand(Indexer.State.EXPECT_RING_FRONT))
            .andThen(
                new WaitUntilCommand(
                    () ->
                        indexer.getState() == Indexer.State.HOLDING_RING
                            || indexer.getState() == Indexer.State.LOST_RING))
            .andThen(transitionCommand(State.TRAVERSING)));

    registerStateCommand(
        State.AMP,
        new SequentialCommandGroup(
            drivetrain.transitionCommand(Drivetrain.State.FIELD_ORIENTED_DRIVE),
            intake.transitionCommand(Intake.State.IDLE),
            new DetermineRingStatusCommand(shooter, indexer, lights),
            shooter.transitionCommand(Shooter.State.AMP),
            new ParallelCommandGroup(
                lightsOnReadyCommand(Lights.State.TARGETING), feedOnPress(State.TRAVERSING))));

    registerStateCommand(
        State.TRAP,
        new SequentialCommandGroup(
            drivetrain.transitionCommand(Drivetrain.State.TRAP),
            new DetermineRingStatusCommand(shooter, indexer, lights),
            shooter.transitionCommand(Shooter.State.TRAP),
            new ParallelCommandGroup(
                lightsOnReadyCommand(Lights.State.TARGETING), feedOnPress(State.TRAVERSING))));

    registerStateCommand(
        State.CLEANSE,
        new SequentialCommandGroup(
            shooter.transitionCommand(Shooter.State.PASS_THROUGH),
            indexer.transitionCommand(Indexer.State.CLEANSE),
            lights.transitionCommand(Lights.State.EJECT),
            new WaitCommand(2),
            transitionCommand(State.TRAVERSING)));

    registerStateCommand(
        State.CLIMB,
        new SequentialCommandGroup(
            drivetrain.transitionCommand(Drivetrain.State.CHAIN_ORIENTED_DRIVE),
            lights.transitionCommand(Lights.State.CLIMB),
            climbers.transitionCommand(Climbers.State.FREE_EXTEND),
            new WaitUntilCommand(controllerBindings.retractClimb()),
            climbers.transitionCommand(Climbers.State.LOADED_RETRACT),
            drivetrain.transitionCommand(Drivetrain.State.X_SHAPE)));

    registerStateCommand(
        State.AUTO_AMP,
        new SequentialCommandGroup(
            drivetrain.transitionCommand(Drivetrain.State.AUTO_AMP),
            new DetermineRingStatusCommand(shooter, indexer, lights),
            shooter.transitionCommand(Shooter.State.AMP),
            drivetrain.waitForState(Drivetrain.State.FIELD_ORIENTED_DRIVE),
            lights.transitionCommand(Lights.State.TARGETING),
            new ParallelCommandGroup(
                lightsOnReadyCommand(Lights.State.TARGETING), feedOnPress(State.TRAVERSING))));

    registerStateCommand(
        State.AUTO_GROUND_INTAKE,
        new ParallelCommandGroup(
            new SequentialCommandGroup(
                drivetrain.transitionCommand(Drivetrain.State.FIELD_ORIENTED_DRIVE),
                new WaitUntilCommand(() -> vision.isFlag(Vision.State.HAS_RING_TARGET)),
                drivetrain.transitionCommand(Drivetrain.State.AUTO_GROUND_INTAKE)),
            new SequentialCommandGroup(
                lights.transitionCommand(Lights.State.INTAKE),
                shooter.transitionCommand(Shooter.State.STOW),
                indexer.transitionCommand(Indexer.State.EXPECT_RING_BACK),
                intake.transitionCommand(Intake.State.INTAKE),
                indexer.waitForState(Indexer.State.INDEXING),
                transitionCommand(State.TRAVERSING))));
  }

  private void registerTransitions() {
    addOmniTransition(State.TRAVERSING);
    addOmniTransition(State.SOFT_E_STOP);
    addOmniTransition(State.SPEAKER_SCORE);
    addOmniTransition(State.BASE_SHOT);
    addOmniTransition(State.HUMAN_PLAYER_INTAKE);
    addOmniTransition(State.AMP);
    addOmniTransition(State.TRAP);
    addOmniTransition(State.CLEANSE);
    addOmniTransition(State.CLIMB);
    addOmniTransition(State.AUTO_AMP);
    addOmniTransition(State.AUTO_GROUND_INTAKE);
    addOmniTransition(State.AUTO_HP_INTAKE);

    addTransition(State.SOFT_E_STOP, State.AUTONOMOUS);
    addTransition(State.TRAVERSING, State.GROUND_INTAKE);
  }

  private Command flashError(Lights.State onEnd) {
    return new SequentialCommandGroup(
        lights.transitionCommand(Lights.State.ERROR),
        new WaitCommand(0.5),
        lights.transitionCommand(onEnd));
  }

  private Command feedOnPress(State onEnd) {
    return new SequentialCommandGroup(
        new WaitUntilCommand(controllerBindings.feedOnPress()),
        indexer.transitionCommand(Indexer.State.FEED_TO_SHOOTER),
        indexer.waitForState(Indexer.State.IDLE),
        new WaitCommand(0.25),
        transitionCommand(onEnd));
  }

  private Command lightsOnReadyCommand(Lights.State alt) {
    return new RunCommand(
        () -> {
          if (shooter.isFlag(Shooter.State.READY)
              && drivetrain.isFlag(Drivetrain.State.AT_ANGLE)
              && indexer.getState() == Indexer.State.HOLDING_RING) {
            lights.requestTransition(Lights.State.READY);
          } else {
            lights.requestTransition(alt);
          }
        });
  }

  private Trigger tuningIncrement() {
    return controllerBindings.tuningIncrement();
  }

  private Trigger tuningDecrement() {
    return controllerBindings.tuningDecrement();
  }

  private Trigger tuningStop() {
    return controllerBindings.tuningStop();
  }

  private void configureBindings() {

    controllerBindings.resetGyro().onTrue(drivetrain.resetGyro());

    controllerBindings
        .xShape()
        .onTrue(
            new InstantCommand(
                    () -> {
                      prevDTState = drivetrain.getState();
                    })
                .andThen(drivetrain.transitionCommand(Drivetrain.State.X_SHAPE)))
        .onFalse(new InstantCommand(() -> drivetrain.requestTransition(prevDTState)));
    controllerBindings
        .shoot()
        .onTrue(
            new ConditionalCommand(
                transitionCommand(State.SPEAKER_SCORE, false),
                transitionCommand(State.BASE_SHOT, false),
                () -> poseWorking))
        .onFalse(transitionCommand(State.TRAVERSING, false));

    controllerBindings
        .groundIntake()
        .onTrue(
            new ConditionalCommand(
                transitionCommand(State.AUTO_GROUND_INTAKE, false),
                transitionCommand(State.GROUND_INTAKE, false),
                () -> autoIntakeWorking))
        .onFalse(transitionCommand(State.TRAVERSING, false));

    controllerBindings
        .manualGroundIntake()
        .onTrue(transitionCommand(State.GROUND_INTAKE, false))
        .onFalse(transitionCommand(State.TRAVERSING, false));

    controllerBindings.traversing().onTrue(transitionCommand(State.TRAVERSING, false));

    controllerBindings
        .humanPlayerIntake()
        .onTrue(transitionCommand(State.HUMAN_PLAYER_INTAKE, false));

    controllerBindings
        .ampScore()
        .onTrue(
            new ConditionalCommand(
                transitionCommand(State.AUTO_AMP, false),
                transitionCommand(State.AMP, false),
                () -> poseWorking));

    controllerBindings.trapScore().onTrue(transitionCommand(State.TRAP, false));

    controllerBindings.cleanse().onTrue(transitionCommand(State.CLEANSE, false));

    controllerBindings.startClimb().debounce(0.5).onTrue(transitionCommand(State.CLIMB, false));

    controllerBindings
        .targetLeftStage()
        .onTrue(new InstantCommand(() -> setTargetStageSide(StageSide.LEFT)));
    controllerBindings
        .targetCenterStage()
        .onTrue(new InstantCommand(() -> setTargetStageSide(StageSide.CENTER)));
    controllerBindings
        .targetRightStage()
        .onTrue(new InstantCommand(() -> setTargetStageSide(StageSide.RIGHT)));
  }

  private void setTargetStageSide(StageSide newSide) {

    targetStageSide = newSide;

    drivetrain.syncTargetStageSide();
  }

  private ClimberIO getLeftClimberIO() {
    return switch (Constants.currentBuildMode) {
      case REAL -> new ClimberIOReal(
          Constants.Climbers.Hardware.LEFT_CLIMBER_ID, Constants.Climbers.Hardware.LEFT_INVERTED);
      case SIM -> new ClimberIOSim(
          Constants.Climbers.Hardware.LEFT_CLIMBER_ID, Constants.Climbers.Hardware.LEFT_INVERTED);
      default -> new ClimberIO() {};
    };
  }

  private ClimberIO getRightClimberIO() {
    return switch (Constants.currentBuildMode) {
      case REAL -> new ClimberIOReal(
          Constants.Climbers.Hardware.RIGHT_CLIMBER_ID, Constants.Climbers.Hardware.RIGHT_INVERTED);
      case SIM -> new ClimberIOSim(
          Constants.Climbers.Hardware.RIGHT_CLIMBER_ID, Constants.Climbers.Hardware.RIGHT_INVERTED);
      default -> new ClimberIO() {};
    };
  }

  private IntakeIO getIntakeIO(BooleanSupplier simProx1) {
    switch (Constants.currentBuildMode) {
      case REAL -> {
        return new IntakeIOReal();
      }
      case SIM -> {
        return new IntakeIOSim(simProx1);
      }
      default -> {
        return new IntakeIO() {};
      }
    }
  }

  private final IndexerIO getIndexerIO(
      BooleanSupplier simProx1, BooleanSupplier simProx2, BooleanSupplier simProx3) {
    switch (Constants.currentBuildMode) {
      case REAL -> {
        return new IndexerIOReal();
      }
      case SIM -> {
        return new IndexerIOSim(simProx1, simProx2, simProx3);
      }
      default -> {
        return new IndexerIO() {};
      }
    }
  }

  private FlywheelIO getFlywheelIO() {
    switch (Constants.currentBuildMode) {
      case REAL -> {
        return new FlywheelIOReal();
      }
      case SIM -> {
        return new FlywheelIOSim();
      }
      default -> {
        return new FlywheelIO() {};
      }
    }
  }

  private ArmIO getArmIO() {
    switch (Constants.currentBuildMode) {
      case REAL -> {
        return new ArmIOReal();
      }
      case SIM -> {
        return new ArmIOSim();
      }
      default -> {
        return new ArmIO() {};
      }
    }
  }

  private LightsIO getLightsIO() {
    switch (Constants.currentBuildMode) {
      case REAL -> {
        return new LightsIOReal();
      }

      default -> {
        return new LightsIO() {};
      }
    }
  }

  @Override
  protected void onEnable() {}

  @Override
  protected void onTeleopStart() {
    requestTransition(State.TRAVERSING);
  }

  @Override
  protected void onAutonomousStart() {

    Command selectedAutoCommand = autoChooser.get();

    registerStateCommand(
        State.AUTONOMOUS,
        new SequentialCommandGroup(
            lights.transitionCommand(Lights.State.AUTO),
            shooter.transitionCommand(Shooter.State.BASE_SHOT),
            shooter.waitForFlag(Shooter.State.READY).withTimeout(1.5),
            indexer.transitionCommand(Indexer.State.FEED_TO_SHOOTER, false),
            indexer.waitForState(Indexer.State.IDLE),
            shooter.transitionCommand(Shooter.State.SPEAKER_AA),
            drivetrain.transitionCommand(Drivetrain.State.FOLLOWING_AUTONOMOUS_TRAJECTORY),
            new InstantCommand(
                () -> {
                  selectedAutoCommand.schedule();
                })));

    requestTransition(State.AUTONOMOUS);
  }

  public void resetFieldOriented() {
    drivetrain.resetFieldOriented();
  }

  public void returnLightsToIdle() {
    lights.requestTransition(Lights.State.RESTING);
  }

  @Override
  protected void determineSelf() {
    setState(State.SOFT_E_STOP);
  }

  public double getShooterAngle() {
    return shooter.getArmAngle();
  }

  public Pose3d getBotPose() {
    Pose2d pose = drivetrain.getBotPose();
    return new Pose3d(
        new Translation3d(pose.getX(), pose.getY(), 0),
        new Rotation3d(0, 0, pose.getRotation().getRadians()));
  }

  public boolean autoReady() {
    return false;
  }

  private void initializeDriveTab() {
    ShuffleboardTab autoTab = Shuffleboard.getTab(Constants.Controller.AUTO_SHUFFLEBOARD_TAB);
    ShuffleboardTab teleTab = Shuffleboard.getTab(Constants.Controller.TELE_SHUFFLEBOARD_TAB_ID);
    ShuffleboardTab testTab = Shuffleboard.getTab(Constants.Controller.TEST_SHUFFLEBOARD_TAB_ID);

    autoTab.add("Auto Route", autoChooser.getSendableChooser()).withPosition(2, 0).withSize(2, 1);

    autoTab
        .addString("ALLIANCE", () -> AllianceManager.getAlliance().name())
        .withPosition(0, 0)
        .withSize(2, 1);
    autoTab
        .add("SWITCH ALLIANCE", AllianceManager.switchAlliance())
        .withPosition(0, 1)
        .withSize(2, 1);
    autoTab.add("SYNC ALLIANCE", AllianceManager.syncAlliance()).withPosition(0, 2).withSize(2, 1);

    // Auto condition checks
    autoTab
        .addNumber("arm absolute", () -> Math.toDegrees(shooter.getArmAbsoluteAngle()))
        .withPosition(5, 0)
        .withSize(1, 1);

    autoTab
        .addNumber("arm relative", () -> Math.toDegrees(shooter.getArmAngle()))
        .withPosition(6, 0)
        .withSize(1, 1);

    autoTab
        .addBoolean(
            "shooter good",
            () ->
                Constants.doubleEqual(
                    shooter.getArmAbsoluteAngle(),
                    shooter.getArmAngle(),
                    Constants.Arm.Settings.AUTO_SYNC_TOLERANCE))
        .withPosition(7, 0)
        .withSize(1, 1);

    autoTab
        .addBoolean("pv good", () -> !vision.isFlag(Vision.State.PV_INSTANCE_DISCONNECT))
        .withPosition(7, 1)
        .withSize(1, 1);

    autoTab
        .addNumber("ll latency", () -> vision.getLimelightLatency())
        .withPosition(5, 2)
        .withSize(1, 1);

    autoTab
        .addNumber("ll offset", () -> vision.getLimelightTargetOffset().getDegrees())
        .withPosition(6, 2)
        .withSize(1, 1);

    autoTab
        .addBoolean(
            "ll good",
            () ->
                Constants.doubleEqual(
                    vision.getLimelightTargetOffset().getDegrees(),
                    0,
                    Constants.Vision.Settings.AUTO_START_TOLERANCE))
        .withPosition(7, 2)
        .withSize(1, 1);

    autoTab
        .addBoolean("prox 1 good", () -> indexer.isProx1Active())
        .withPosition(5, 3)
        .withSize(1, 1);
    autoTab
        .addBoolean("prox 2 good", () -> indexer.isProx2Active())
        .withPosition(6, 3)
        .withSize(1, 1);
    autoTab
        .addBoolean("prox 3 good", () -> !indexer.isProx3Active())
        .withPosition(7, 3)
        .withSize(1, 1);

    autoTab.addBoolean("GOOD TO GO", this::autoReady).withPosition(9, 0).withSize(3, 3);

    teleTab.addBoolean("POSE WORKING", () -> poseWorking).withPosition(0, 0).withSize(2, 2);
    teleTab.addBoolean("AUTO INTAKE", () -> autoIntakeWorking).withPosition(2, 0).withSize(2, 2);

    teleTab
        .add(
            "TOGGLE POSE",
            new WhileDisabledInstantCommand(
                () -> {
                  poseWorking = !poseWorking;
                }))
        .withPosition(0, 2)
        .withSize(2, 1);
    teleTab
        .add(
            "TOGGLE INTAKE",
            new WhileDisabledInstantCommand(
                () -> {
                  autoIntakeWorking = !autoIntakeWorking;
                }))
        .withPosition(2, 2)
        .withSize(2, 1);

    teleTab
        .addBoolean("SHOOTER READY", () -> shooter.isFlag(Shooter.State.READY))
        .withSize(3, 3)
        .withPosition(8, 1);

    teleTab.addBoolean("HAVE RING", () -> indexer.ringPresent()).withSize(3, 3).withPosition(5, 1);

    testTab
        .add("zero climbers", new InstantCommand(() -> climbers.zero()))
        .withPosition(5, 0)
        .withSize(2, 2);

    testTab
        .addNumber("climber left", () -> climbers.getLeftPos())
        .withPosition(3, 0)
        .withSize(2, 1);

    testTab
        .addNumber("climber right", () -> climbers.getRightPos())
        .withPosition(3, 0)
        .withSize(2, 1);

    Shuffleboard.selectTab(Constants.Controller.AUTO_SHUFFLEBOARD_TAB);
  }

  public enum State {
    UNDETERMINED,
    AUTONOMOUS,
    TRAVERSING,
    SPEAKER_SCORE,
    BASE_SHOT,
    SOFT_E_STOP,
    CLIMB,
    AMP,
    AUTO_AMP,
    GROUND_INTAKE,
    HUMAN_PLAYER_INTAKE,
    AUTO_GROUND_INTAKE,
    AUTO_HP_INTAKE,
    TRAP,
    CLEANSE
  }
}
