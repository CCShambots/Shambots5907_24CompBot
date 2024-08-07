// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.Vision.Settings.LEFT_INTAKE_CAM_SETTINGS;
import static frc.robot.Constants.Vision.Settings.LEFT_SHOOTER_CAM_SETTINGS;
import static frc.robot.Constants.Vision.Settings.RIGHT_INTAKE_CAM_SETTINGS;
import static frc.robot.Constants.Vision.Settings.RIGHT_SHOOTER_CAM_SETTINGS;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
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
import frc.robot.controllers.BartaSimBindings;
import frc.robot.controllers.ControllerBindings;
import frc.robot.controllers.RealControllerBindings;
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
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer extends StateMachine<RobotContainer.State> {

  // Declare Subsystems
  private final Intake intake;
  private final Shooter shooter;
  private final Indexer indexer;
  private final Vision vision;
  private final Climbers climbers;
  private final Drivetrain drivetrain;
  private final Lights lights;

  // Declare power distribution for voltage and current monitoring/logging
  private PowerDistribution pd;

  // Controller bindings object that will be created to handle both real control
  // and sim inputs
  private final ControllerBindings controllerBindings;

  private final LoggedDashboardChooser<Command> autoChooser;

  // Store the previous drivetrain state so that x-shape command works properly
  private Drivetrain.State prevDTState = Drivetrain.State.FIELD_ORIENTED_DRIVE;

  // Configuration variables that can be adjusted in game
  @AutoLogOutput private StageSide targetStageSide = StageSide.CENTER;
  private boolean poseWorking = true;
  private boolean autoIntakeWorking = true;
  private Shooter.State autoLobState = Shooter.State.LOB_ACTIVE_ADJUST;

  // Track whether the bot has been enabled for autonomous check lights
  private boolean hasBeenEnabled = false;

  private double closeFourNoteDelay = 0;
  private GenericEntry delaySlider;

  private GenericEntry tuningHoodAngle;
  private GenericEntry tuningFlywheelSpeed;

  public RobotContainer(EventLoop checkModulesLoop, PowerDistribution pd) {
    super("RobotContainer", State.UNDETERMINED, State.class);

    this.pd = pd;

    if (Constants.currentBuildMode == ShamLibConstants.BuildMode.SIM) {
      controllerBindings = new BartaSimBindings();
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

    List<Vision.CamSettings> photonArray =
        Constants.currentBuildMode == BuildMode.SIM
            ? List.of()
            : List.of(
                Constants.Vision.Settings.LEFT_SHOOTER_CAM_SETTINGS,
                Constants.Vision.Settings.RIGHT_SHOOTER_CAM_SETTINGS,
                Constants.Vision.Settings.RIGHT_INTAKE_CAM_SETTINGS,
                Constants.Vision.Settings.LEFT_INTAKE_CAM_SETTINGS);

    vision = new Vision("limelight", photonArray);

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

    vision.setOverallEstimateSupplier(drivetrain::getBotPose);

    climbers =
        new Climbers(
            getLeftClimberIO(),
            getRightClimberIO(),
            tuningIncrement(),
            tuningDecrement(),
            tuningStop());

    lights =
        new Lights(
            getLightsIO(),
            () -> !hasBeenEnabled,
            intake::ringPresent,
            climbers::isLeftTouchTripped,
            climbers::isRightTouchTripped,
            autoConditions());

    shooter =
        new Shooter(
            getArmIO(),
            getFlywheelIO(),
            () -> drivetrain.getBotPose().getTranslation(),
            () -> drivetrain.getMovingSpeakerShootPose().getTranslation(),
            () -> Math.toRadians(tuningHoodAngle.getDouble(0)),
            () -> tuningFlywheelSpeed.getDouble(0) / 60.0,
            tuningIncrement(),
            tuningDecrement(),
            tuningStop());

    addChildSubsystem(drivetrain);
    addChildSubsystem(vision);
    addChildSubsystem(intake);
    addChildSubsystem(shooter);
    addChildSubsystem(indexer);
    addChildSubsystem(climbers);

    // Since the lights subsystem should always be able to change state, it should
    // be enabled
    // immediately
    // That means it's also not a child subsystem of the RobotContainer, because
    // otherwise it would
    // get disabled
    // when it shouldn't
    lights.enable();

    registerStateCommands();
    registerTransitions();

    configureTriggerBindings();

    registerNamedCommands();

    // Pathplanner must be configured AFTER all the named commands are set,
    // otherwise the code will
    // crash
    drivetrain.configurePathplanner();

    // Important to instatiate after drivetrain consructor is called so that auto
    // builder is
    // configured
    autoChooser =
        new LoggedDashboardChooser<>("Logged Autonomous Chooser", AutoBuilder.buildAutoChooser());

    initializeShuffleboardTabs();

    // Make sure to turn off controller rumble in case it is still rumbling by
    // chance
    controllerBindings.setRumble(0);
  }

  private void registerNamedCommands() {
    NamedCommands.registerCommand("raiseClimber", new InstantCommand());

    NamedCommands.registerCommand(
        "intake",
        new ParallelCommandGroup(
            intake.transitionCommand(Intake.State.INTAKE, false),
            new ConditionalCommand(
                indexer.transitionCommand(Indexer.State.EXPECT_RING_BACK),
                new InstantCommand(
                    () ->
                        new SequentialCommandGroup(
                                indexer.waitForState(Indexer.State.IDLE).withTimeout(2),
                                indexer.transitionCommand(Indexer.State.EXPECT_RING_BACK))
                            .schedule()),
                () -> indexer.getState() == Indexer.State.IDLE)));

    NamedCommands.registerCommand("stopIntake", intake.transitionCommand(Intake.State.IDLE, false));

    NamedCommands.registerCommand(
        "shoot",
        new SequentialCommandGroup(
            indexer.waitForState(Indexer.State.HOLDING_RING).withTimeout(3),
            indexer.transitionCommand(Indexer.State.FEED_TO_SHOOTER, false)));

    NamedCommands.registerCommand(
        "slowDownShooterForClown",
        new ParallelCommandGroup(
            indexer.transitionCommand(Indexer.State.BLIND_FEED),
            shooter.transitionCommand(Shooter.State.PASS_THROUGH)));

    NamedCommands.registerCommand(
        "backToNormalShooting",
        new ParallelCommandGroup(
            indexer.transitionCommand(Indexer.State.IDLE),
            shooter.transitionCommand(Shooter.State.SPEAKER_AA)));

    NamedCommands.registerCommand(
        "rawVisionIntake",
        new ParallelCommandGroup(indexer.transitionCommand(Indexer.State.BLIND_FEED)));

    NamedCommands.registerCommand(
        "disablePassThrough",
        new ParallelCommandGroup(indexer.transitionCommand(Indexer.State.IDLE)));

    NamedCommands.registerCommand(
        "enableLob", shooter.transitionCommand(Shooter.State.LOB_STRAIGHT));
    NamedCommands.registerCommand(
        "disableLob", shooter.transitionCommand(Shooter.State.SPEAKER_AA));

    NamedCommands.registerCommand(
        "shortDrop",
        new SequentialCommandGroup(
            shooter.transitionCommand(Shooter.State.PASS_THROUGH),
            indexer.transitionCommand(Indexer.State.FEED_TO_SHOOTER)));

    NamedCommands.registerCommand(
        "spinUpAgain", shooter.transitionCommand(Shooter.State.SPEAKER_AA));

    NamedCommands.registerCommand(
        "fireSequence",
        new ConditionalCommand(
            new SequentialCommandGroup(
                indexer.waitForState(Indexer.State.HOLDING_RING).withTimeout(1.5),
                intake.transitionCommand(Intake.State.IDLE, false),
                indexer.transitionCommand(Indexer.State.FEED_TO_SHOOTER, false),
                new ParallelCommandGroup(
                    intake.transitionCommand(Intake.State.INTAKE, false),
                    new InstantCommand(
                        () -> {
                          new SequentialCommandGroup(
                                  indexer.waitForState(Indexer.State.IDLE),
                                  indexer.transitionCommand(Indexer.State.EXPECT_RING_BACK))
                              .schedule();
                        }))),
            new InstantCommand(),
            () -> ringSomewhereInBot()));

    NamedCommands.registerCommand(
        "visionIntake",
        new SequentialCommandGroup(
            drivetrain.transitionCommand(Drivetrain.State.AUTO_GROUND_INTAKE),
            drivetrain.waitForFlag(Drivetrain.State.AUTO_INTAKING),
            indexer
                .waitForState(Indexer.State.INDEXING)
                .raceWith(indexer.waitForState(Indexer.State.HOLDING_RING))
                .raceWith(
                    new WaitUntilCommand(() -> !drivetrain.isFlag(Drivetrain.State.AUTO_INTAKING)))
                .withTimeout(3),
            drivetrain.transitionCommand(Drivetrain.State.IDLE),
            drivetrain.transitionCommand(Drivetrain.State.FOLLOWING_AUTONOMOUS_TRAJECTORY)));

    NamedCommands.registerCommand(
        "feedVisionIntake",
        new SequentialCommandGroup(
            indexer.waitForState(Indexer.State.IDLE),
            indexer.transitionCommand(Indexer.State.PASS_THROUGH),
            intake.transitionCommand(Intake.State.INTAKE),
            drivetrain.transitionCommand(Drivetrain.State.AUTO_GROUND_INTAKE),
            drivetrain.waitForFlag(Drivetrain.State.AUTO_INTAKING),
            new SequentialCommandGroup(
                    new WaitUntilCommand(() -> intake.ringPresent()),
                    new WaitUntilCommand(() -> !indexer.ringPresent()))
                .withTimeout(3),
            drivetrain.transitionCommand(Drivetrain.State.IDLE),
            drivetrain.transitionCommand(Drivetrain.State.FOLLOWING_AUTONOMOUS_TRAJECTORY)));

    NamedCommands.registerCommand(
        "dumbVisionIntake",
        new SequentialCommandGroup(
            drivetrain.transitionCommand(Drivetrain.State.AUTO_GROUND_INTAKE),
            drivetrain.waitForFlag(Drivetrain.State.AUTO_INTAKING),
            new WaitUntilCommand(() -> intake.ringPresent()),
            new WaitCommand(0.25),
            new WaitUntilCommand(() -> !indexer.ringPresent()),
            drivetrain.transitionCommand(Drivetrain.State.IDLE),
            drivetrain.transitionCommand(Drivetrain.State.FOLLOWING_AUTONOMOUS_TRAJECTORY)));

    NamedCommands.registerCommand(
        "aim",
        new SequentialCommandGroup(
            drivetrain.transitionCommand(Drivetrain.State.FACE_SPEAKER_AUTO),
            new WaitCommand(0.25),
            new ParallelCommandGroup(
                    drivetrain.waitForFlag(Drivetrain.State.AT_ANGLE),
                    shooter.waitForFlag(Shooter.State.READY))
                .withTimeout(0.25),
            drivetrain.transitionCommand(Drivetrain.State.IDLE),
            drivetrain.transitionCommand(Drivetrain.State.FOLLOWING_AUTONOMOUS_TRAJECTORY)));

    NamedCommands.registerCommand(
        "aimLong",
        new SequentialCommandGroup(
            drivetrain.transitionCommand(Drivetrain.State.FACE_SPEAKER_AUTO),
            new WaitCommand(1.25),
            drivetrain.transitionCommand(Drivetrain.State.IDLE),
            drivetrain.transitionCommand(Drivetrain.State.FOLLOWING_AUTONOMOUS_TRAJECTORY)));

    NamedCommands.registerCommand(
        "enableAimWhileMove",
        new SequentialCommandGroup(
            drivetrain.transitionCommand(Drivetrain.State.FOLLOWING_AUTONOMOUS_TRAJECTORY_AIMING),
            shooter.transitionCommand(Shooter.State.MOVING_SPEAKER_AA)));
    NamedCommands.registerCommand(
        "disableAimWhileMove",
        new SequentialCommandGroup(
            drivetrain.transitionCommand(Drivetrain.State.FOLLOWING_AUTONOMOUS_TRAJECTORY),
            shooter.transitionCommand(Shooter.State.SPEAKER_AA)));
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
            new ConditionalCommand(
                intake.transitionCommand(Intake.State.IDLE),
                new SequentialCommandGroup(
                    indexer.waitForState(Indexer.State.HOLDING_RING),
                    intake.transitionCommand(Intake.State.IDLE)),
                () -> indexer.getState() != Indexer.State.INDEXING),
            vision.transitionCommand(Vision.State.ENABLED),
            new SequentialCommandGroup(
                indexer.waitForState(Indexer.State.HOLDING_RING),
                lights.transitionCommand(Lights.State.HAVE_RING)),
            new SequentialCommandGroup(
                new DetermineRingStatusCommand(shooter, indexer, lights),
                shooter.partialFlywheelSpinup())));

    registerStateCommand(
        State.TEST, new ParallelCommandGroup(lights.transitionCommand(Lights.State.TEST)));

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
            // lights show green on ready and feed ring on press, transition to traversing
            // after
            // ring is fed
            new ParallelCommandGroup(
                lightsOnReadyCommand(Lights.State.TARGETING), feedOnPress(State.TRAVERSING))));

    registerStateCommand(
        State.LOB,
        new SequentialCommandGroup(
            // face speaker and idle intake
            new ConditionalCommand(
                drivetrain.transitionCommand(Drivetrain.State.LOB),
                drivetrain.transitionCommand(Drivetrain.State.FIELD_ORIENTED_DRIVE),
                () -> poseWorking),
            intake.transitionCommand(Intake.State.IDLE),
            // figure out ring issues (if there are any)
            new DetermineRingStatusCommand(shooter, indexer, lights),
            // have shooter start to track
            new InstantCommand(
                () -> {
                  shooter.requestTransition(autoLobState);
                }),
            new ParallelCommandGroup(
                new RunCommand(
                    () -> {
                      if (shooter.getState() != autoLobState && !shooter.isTransitioning()) {
                        shooter.requestTransition(autoLobState);
                      }
                    }),
                // lights show green on ready and feed ring on press, transition to traversing
                // after
                // ring is fed
                lightsOnReadyCommand(Lights.State.TARGETING),
                feedOnPress(State.TRAVERSING, false))));

    registerStateCommand(
        State.GROUND_INTAKE,
        new SequentialCommandGroup(
            new ParallelCommandGroup(
                // drivetrain.transitionCommand(Drivetrain.State.GROUND_INTAKE),
                shooter.transitionCommand(Shooter.State.STOW),
                indexer.transitionCommand(Indexer.State.EXPECT_RING_BACK),
                lights.transitionCommand(Lights.State.INTAKE),
                intake.transitionCommand(Intake.State.INTAKE)),
            indexer.waitForState(Indexer.State.INDEXING),
            new ConditionalCommand(
                transitionCommand(State.TRAVERSING),
                Commands.none(),
                () -> getState() == State.GROUND_INTAKE)));

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
        new SequentialCommandGroup(
            new ParallelCommandGroup(
                drivetrain.transitionCommand(Drivetrain.State.FIELD_ORIENTED_DRIVE),
                intake.transitionCommand(Intake.State.IDLE),
                lights.transitionCommand(Lights.State.GRAB_RANDOM_NOTE),
                shooter.transitionCommand(Shooter.State.CHUTE_INTAKE),
                indexer.transitionCommand(Indexer.State.EXPECT_RING_FRONT)),
            new ParallelCommandGroup(
                new ConditionalCommand(
                    new SequentialCommandGroup(
                        new WaitUntilCommand(drivetrain::closeEnoughForSourceAlign),
                        drivetrain.transitionCommand(Drivetrain.State.SOURCE_INTAKE)),
                    new InstantCommand(),
                    () -> poseWorking),
                new SequentialCommandGroup(
                    new WaitUntilCommand(
                        () ->
                            indexer.getState() == Indexer.State.HOLDING_RING
                                || indexer.getState() == Indexer.State.LOST_RING),
                    new InstantCommand(
                        () -> {
                          new WaitCommand(1)
                              .andThen(indexer.transitionCommand(Indexer.State.INDEXING, false))
                              .schedule();
                        }),
                    transitionCommand(State.TRAVERSING)))));

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
                new ConditionalCommand(
                    new SequentialCommandGroup(
                        new WaitUntilCommand(drivetrain::closeEnoughForAmpAlign),
                        drivetrain.transitionCommand(Drivetrain.State.FACE_AMP)),
                    new InstantCommand(),
                    () -> poseWorking),
                lightsOnReadyCommand(Lights.State.TARGETING),
                feedOnPress(State.TRAVERSING, false))));

    registerStateCommand(
        State.TRAP,
        new SequentialCommandGroup(
            vision.transitionCommand(Vision.State.TRAP),
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
            intake.transitionCommand(Intake.State.INTAKE),
            lights.transitionCommand(Lights.State.EJECT),
            new WaitCommand(2),
            transitionCommand(State.TRAVERSING)));

    registerStateCommand(
        State.EJECT_INTAKE,
        new SequentialCommandGroup(
            intake.transitionCommand(Intake.State.EXPEL),
            lights.transitionCommand(Lights.State.EJECT),
            new WaitCommand(2),
            transitionCommand(State.TRAVERSING)));

    registerStateCommand(
        State.CLIMB,
        new ParallelCommandGroup(
            new ConditionalCommand(
                drivetrain.transitionCommand(Drivetrain.State.CHAIN_ORIENTED_DRIVE),
                Commands.none(),
                () -> poseWorking),
            lights.transitionCommand(Lights.State.CLIMB),
            shooter.flywheelSpinDown(),
            climbers.transitionCommand(Climbers.State.FREE_EXTEND),
            new SequentialCommandGroup(
                new WaitUntilCommand(controllerBindings.retractClimb()),
                climbers.transitionCommand(Climbers.State.LOADED_RETRACT),
                drivetrain.transitionCommand(Drivetrain.State.X_SHAPE)),
            new SequentialCommandGroup(
                new PrintCommand("Hello"),
                new WaitUntilCommand(controllerBindings.startClimb().negate()),
                new PrintCommand("Hello2"),
                new WaitUntilCommand(controllerBindings.startClimb()),
                new PrintCommand("Hello3"),
                transitionCommand(State.CENTERED_CLIMB, false))));

    registerStateCommand(
        State.CENTERED_CLIMB,
        new ParallelCommandGroup(
            new ConditionalCommand(
                drivetrain.transitionCommand(Drivetrain.State.CHAIN_ALIGNED_DRIVE),
                Commands.none(),
                () -> poseWorking),
            lights.transitionCommand(Lights.State.CENTERED_CLIMB),
            shooter.flywheelSpinDown(),
            climbers.transitionCommand(Climbers.State.MINIMUM_EXTEND),
            new SequentialCommandGroup(
                new WaitUntilCommand(controllerBindings.retractClimb()),
                climbers.transitionCommand(Climbers.State.LOADED_RETRACT),
                drivetrain.transitionCommand(Drivetrain.State.X_SHAPE)),
            new SequentialCommandGroup(
                new PrintCommand("Hello"),
                new WaitUntilCommand(controllerBindings.startClimb().negate()),
                new PrintCommand("Hello2"),
                new WaitUntilCommand(controllerBindings.startClimb()),
                new PrintCommand("Hello3"),
                transitionCommand(State.CLIMB, false))));

    registerStateCommand(
        State.AUTO_AMP,
        new SequentialCommandGroup(
            drivetrain.transitionCommand(Drivetrain.State.AUTO_AMP),
            new DetermineRingStatusCommand(shooter, indexer, lights),
            shooter.transitionCommand(Shooter.State.AMP),
            drivetrain.waitForState(Drivetrain.State.FIELD_ORIENTED_DRIVE),
            lights.transitionCommand(Lights.State.TARGETING),
            new ParallelCommandGroup(
                lightsOnReadyCommand(Lights.State.TARGETING),
                feedOnPress(State.TRAVERSING, false))));

    registerStateCommand(
        State.AUTO_GROUND_INTAKE,
        new ParallelCommandGroup(
            new SequentialCommandGroup(
                    drivetrain.transitionCommand(Drivetrain.State.FIELD_ORIENTED_DRIVE),
                    new WaitUntilCommand(() -> vision.isFlag(Vision.State.HAS_RING_TARGET)),
                    drivetrain.transitionCommand(Drivetrain.State.AUTO_GROUND_INTAKE),
                    new WaitUntilCommand(() -> !vision.isFlag(Vision.State.HAS_RING_TARGET)),
                    flashError(Lights.State.INTAKE))
                .repeatedly(),
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                    lights.transitionCommand(Lights.State.INTAKE),
                    shooter.transitionCommand(Shooter.State.STOW),
                    indexer.transitionCommand(Indexer.State.EXPECT_RING_BACK),
                    intake.transitionCommand(Intake.State.INTAKE)),
                indexer.waitForState(Indexer.State.INDEXING),
                new ConditionalCommand(
                    transitionCommand(State.TRAVERSING),
                    new InstantCommand(),
                    () -> getState() == State.AUTO_GROUND_INTAKE))));
  }

  private void registerTransitions() {
    addOmniTransition(State.SPEAKER_SCORE);
    addOmniTransition(State.BASE_SHOT);
    addOmniTransition(State.HUMAN_PLAYER_INTAKE);
    addOmniTransition(State.AMP);
    addOmniTransition(State.TRAP);
    addOmniTransition(State.CLEANSE);
    addOmniTransition(State.EJECT_INTAKE);
    addOmniTransition(State.AUTO_AMP);
    addOmniTransition(State.AUTO_GROUND_INTAKE);
    addOmniTransition(State.AUTO_HP_INTAKE);
    addOmniTransition(State.LOB);

    // Make sure we can't enter other states from the climb state
    removeAllTransitionsFromState(State.CLIMB);
    addTransition(State.TRAVERSING, State.CLIMB);
    addTransition(State.CLIMB, State.CENTERED_CLIMB);
    addTransition(State.CENTERED_CLIMB, State.CLIMB);

    // Make sure we can't enter other states from the trap state
    removeAllTransitionsFromState(State.TRAP);
    addTransition(State.TRAVERSING, State.TRAP);

    addTransition(State.SOFT_E_STOP, State.AUTONOMOUS);
    addTransition(State.SOFT_E_STOP, State.TEST);
    addTransition(State.TRAVERSING, State.GROUND_INTAKE);

    addOmniTransition(State.SOFT_E_STOP);
    addOmniTransition(State.TRAVERSING);
  }

  private Command flashError(Lights.State onEnd) {
    return flash(Lights.State.ERROR, onEnd, 0.5);
  }

  private Command flash(Lights.State flashState, Lights.State onEnd, double duration) {
    return new SequentialCommandGroup(
        lights.transitionCommand(flashState), new WaitCommand(1), lights.transitionCommand(onEnd));
  }

  /**
   * Command to feed the note through the flywheels when the shooter is ready and a button is
   * pressed
   */
  private Command feedOnPress(State onEnd, boolean useDelay) {
    return new SequentialCommandGroup(
        new WaitUntilCommand(
            () ->
                controllerBindings.feedOnPress().getAsBoolean()
                    && shooter.isFlag(Shooter.State.READY)),
        new ConditionalCommand(
            new ConditionalCommand(
                Commands.none(),
                new WaitCommand(0.5),
                () -> drivetrain.isCLoseEnoughToBypassWait(vision)),
            new InstantCommand(),
            () -> useDelay),
        indexer.transitionCommand(Indexer.State.FEED_TO_SHOOTER),
        indexer.waitForState(Indexer.State.IDLE),
        transitionCommand(onEnd));
  }

  private Command feedOnPress(State onEnd) {
    return feedOnPress(onEnd, true);
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

  public void alignSwerveModules() {
    drivetrain.alignModules();
  }

  private void configureTriggerBindings() {

    controllerBindings.resetGyro().onTrue(drivetrain.resetFieldOrientedTele());

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

    controllerBindings.manualBaseShot().onTrue(transitionCommand(State.BASE_SHOT, false));

    controllerBindings
        .groundIntake()
        .onTrue(
            new ConditionalCommand(
                transitionCommand(State.AUTO_GROUND_INTAKE, false),
                transitionCommand(State.GROUND_INTAKE, false),
                () -> autoIntakeWorking))
        .onFalse(
            new ConditionalCommand(
                transitionCommand(State.TRAVERSING, false),
                Commands.none(),
                () -> getState() == State.AUTO_GROUND_INTAKE || getState() == State.GROUND_INTAKE));

    controllerBindings
        .manualGroundIntake()
        .onTrue(transitionCommand(State.GROUND_INTAKE, false))
        .onFalse(
            new ConditionalCommand(
                transitionCommand(State.TRAVERSING, false),
                Commands.none(),
                () -> getState() == State.GROUND_INTAKE));

    controllerBindings.traversing().onTrue(transitionCommand(State.TRAVERSING, false));

    controllerBindings
        .humanPlayerIntake()
        .onTrue(transitionCommand(State.HUMAN_PLAYER_INTAKE, false));

    controllerBindings
        .autoAmp()
        .onTrue(
            new ConditionalCommand(
                transitionCommand(State.AUTO_AMP, false),
                transitionCommand(State.AMP, false),
                () -> poseWorking))
        .onFalse(transitionCommand(State.TRAVERSING));

    controllerBindings.manualAmp().onTrue(transitionCommand(State.AMP, false));

    controllerBindings
        .trapScore()
        .and(() -> poseWorking)
        .onTrue(transitionCommand(State.TRAP, false));

    controllerBindings.cleanse().onTrue(transitionCommand(State.CLEANSE, false));
    controllerBindings.ejectIntake().onTrue(transitionCommand(State.EJECT_INTAKE, false));

    controllerBindings.startClimb().onTrue(transitionCommand(State.CLIMB, false));

    controllerBindings
        .preExtendClimbers()
        .onTrue(climbers.transitionCommand(Climbers.State.FREE_EXTEND));

    controllerBindings
        .targetLeftStage()
        .onTrue(new InstantCommand(() -> setTargetStageSide(StageSide.LEFT)));
    controllerBindings
        .targetCenterStage()
        .onTrue(new InstantCommand(() -> setTargetStageSide(StageSide.CENTER)));
    controllerBindings
        .targetRightStage()
        .onTrue(new InstantCommand(() -> setTargetStageSide(StageSide.RIGHT)));

    controllerBindings
        .lobShot()
        .onTrue(transitionCommand(State.LOB, false))
        .onFalse(transitionCommand(State.TRAVERSING, false));

    controllerBindings
        .toggleLobMode()
        .onTrue(
            new ParallelCommandGroup(
                toggleLobMode(),
                new InstantCommand(
                    () -> {
                      flash(Lights.State.LOB_TOGGLE, lights.getState(), 1).schedule();
                    })));

    controllerBindings
        .indicateNonSourceNote()
        .and(() -> getState() == State.TRAVERSING)
        .and(() -> !ringSomewhereInBot())
        .onTrue(lights.transitionCommand(Lights.State.GRAB_RANDOM_NOTE));
    controllerBindings
        .indicateSourceNote()
        .and(() -> getState() == State.TRAVERSING)
        .and(() -> !ringSomewhereInBot())
        .onTrue(lights.transitionCommand(Lights.State.NO_RING));

    // Rumble the controller when the voltage drops below a certain value for too
    // long
    new Trigger(this::lowVoltage)
        .debounce(1)
        .onTrue(new InstantCommand(() -> controllerBindings.setRumble(1)))
        .onFalse(new InstantCommand(() -> controllerBindings.setRumble(0)));

    controllerBindings
        .resetVisionPose()
        .onTrue(
            new InstantCommand(
                () -> {
                  System.out.println("RUNNING ATTEMPT TO RESET");
                  Optional<Pose2d> pose =
                      vision.getCurrentVisionPoseEstimate(
                          Constants.Vision.Settings.LEFT_SHOOTER_CAM_SETTINGS.name());

                  // Print the estimate details to the console
                  System.out.println("Pose:" + pose.toString());

                  if (pose.isPresent()) {
                    drivetrain.setPose(pose.get());
                  } else {
                    flashError(lights.getState());
                  }
                }));

    controllerBindings.enterTune().onTrue(shooter.enableTune());
  }

  public boolean lowVoltage() {
    return pd != null && pd.getVoltage() <= Constants.Controller.VOLTAGE_WARNING;
  }

  private void setTargetStageSide(StageSide newSide) {
    targetStageSide = newSide;
  }

  private ClimberIO getLeftClimberIO() {
    return switch (Constants.currentBuildMode) {
      case REAL -> new ClimberIOReal(
          Constants.Climbers.Hardware.LEFT_CLIMBER_ID,
          Constants.Climbers.Hardware.LEFT_INVERTED,
          Constants.Climbers.Hardware.LEFT_TOUCH_ID);
      case SIM -> new ClimberIOSim(
          Constants.Climbers.Hardware.LEFT_CLIMBER_ID,
          Constants.Climbers.Hardware.LEFT_INVERTED,
          Constants.Climbers.Hardware.LEFT_TOUCH_ID);
      default -> new ClimberIO() {};
    };
  }

  private ClimberIO getRightClimberIO() {
    return switch (Constants.currentBuildMode) {
      case REAL -> new ClimberIOReal(
          Constants.Climbers.Hardware.RIGHT_CLIMBER_ID,
          Constants.Climbers.Hardware.RIGHT_INVERTED,
          Constants.Climbers.Hardware.RIGHT_TOUCH_ID);
      case SIM -> new ClimberIOSim(
          Constants.Climbers.Hardware.RIGHT_CLIMBER_ID,
          Constants.Climbers.Hardware.RIGHT_INVERTED,
          Constants.Climbers.Hardware.RIGHT_TOUCH_ID);
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
      default -> {
        return new LightsIOReal();
      }

        // default -> {
        // return new LightsIO() {};
        // }
    }
  }

  @Override
  protected void onEnable() {
    hasBeenEnabled = true;
  }

  @Override
  protected void onDisable() {
    controllerBindings.setRumble(0);
  }

  @Override
  protected void onTeleopStart() {
    requestTransition(State.TRAVERSING);
  }

  @Override
  protected void onAutonomousStart() {

    closeFourNoteDelay = delaySlider.getDouble(closeFourNoteDelay);

    Command selectedAutoCommand = autoChooser.get();

    String selectedAutoKey = autoChooser.getSendableChooser().getSelected();

    AtomicBoolean runningDelayPathfindAuto = new AtomicBoolean(false);

    if (selectedAutoKey.equals("4 Note")) runningDelayPathfindAuto.set(true);
    if (selectedAutoKey.equals("5 Note Adaptive")) runningDelayPathfindAuto.set(true);

    Logger.recordOutput("RobotContainer/AutoKey", selectedAutoKey);

    registerStateCommand(
        State.AUTONOMOUS,
        new SequentialCommandGroup(
            lights.transitionCommand(Lights.State.AUTO),
            shooter.enableRapidSpinup(),
            shooter.transitionCommand(Shooter.State.SPEAKER_AA),
            new ConditionalCommand(
                new SequentialCommandGroup(
                    new WaitCommand(closeFourNoteDelay),
                    drivetrain.transitionCommand(Drivetrain.State.START_CLOSE_4),
                    drivetrain.waitForState(Drivetrain.State.IDLE)),
                new InstantCommand(),
                () -> runningDelayPathfindAuto.get()),
            drivetrain.transitionCommand(Drivetrain.State.FOLLOWING_AUTONOMOUS_TRAJECTORY),
            new InstantCommand(
                () -> {
                  selectedAutoCommand.schedule();
                })));

    requestTransition(State.AUTONOMOUS);
  }

  @Override
  protected void onTestStart() {
    requestTransition(State.TEST);
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

  private Command toggleLobMode() {
    return new InstantCommand(
        () ->
            autoLobState =
                autoLobState == Shooter.State.LOB_STRAIGHT
                    ? Shooter.State.LOB_ACTIVE_ADJUST
                    : Shooter.State.LOB_STRAIGHT);
  }

  /** The conditions required for autonomous to be ready to start */
  public BooleanSupplier[] autoConditions() {
    // Cameras are initialized in the order 1,4,3,2 for some reason
    // This should be fixed in future years but I didn't want to modify it
    return new BooleanSupplier[] {
      () -> !hasBeenEnabled,
      () -> shooterGood(),
      () -> llGood(),
      () -> vision.isConnected(LEFT_SHOOTER_CAM_SETTINGS.name()),
      () -> vision.isConnected(LEFT_INTAKE_CAM_SETTINGS.name()),
      () -> vision.isConnected(RIGHT_INTAKE_CAM_SETTINGS.name()),
      () -> vision.isConnected(RIGHT_SHOOTER_CAM_SETTINGS.name()),
      () -> prox1Good(),
      () -> prox2Good(),
      () -> prox3Good(),
      () -> DriverStation.isDSAttached()
    };
  }

  public boolean autoReady() {
    return shooterGood()
        && photonVisionGood()
        && prox1Good()
        && prox2Good()
        && prox3Good()
        && llGood()
        && DriverStation.isDSAttached();
  }

  private boolean ringSomewhereInBot() {
    return intake.ringPresent()
        || indexer.isProx1Active()
        || indexer.isProx2Active()
        || indexer.isProx3Active();
  }

  /** Shooter absolute and relative angle are close */
  private boolean shooterGood() {
    return Constants.doubleEqual(
        shooter.getArmAbsoluteAngle(),
        shooter.getArmAngle(),
        Constants.Arm.Settings.AUTO_SYNC_TOLERANCE);
  }

  /** All photon vision instances are connected */
  private boolean photonVisionGood() {
    return !vision.isFlag(Vision.State.PV_INSTANCE_DISCONNECT);
  }

  /** */
  private boolean prox1Good() {
    return indexer.isProx1Active();
  }

  private boolean prox2Good() {
    return indexer.isProx2Active();
  }

  private boolean prox3Good() {
    return !indexer.isProx3Active();
  }

  private boolean llGood() {
    return vision.getLimelightLatency() > 25 && vision.getLimelightLatency() < 100;
  }

  private void initializeShuffleboardTabs() {
    ShuffleboardTab autoTab = Shuffleboard.getTab(Constants.Controller.AUTO_SHUFFLEBOARD_TAB);
    ShuffleboardTab teleTab = Shuffleboard.getTab(Constants.Controller.TELE_SHUFFLEBOARD_TAB_ID);
    ShuffleboardTab testTab = Shuffleboard.getTab(Constants.Controller.TEST_SHUFFLEBOARD_TAB_ID);
    ShuffleboardTab tuningTab = Shuffleboard.getTab(Constants.Controller.TUNE_SHUFFLEBOARD_TAB_ID);

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

    autoTab.addBoolean("shooter good", this::shooterGood).withPosition(7, 0).withSize(1, 1);

    autoTab.add("SYNC ARM", shooter.syncAbsoluteAngle()).withPosition(4, 0).withSize(1, 1);

    autoTab
        .addBoolean("pv 1 good", () -> vision.isConnected(LEFT_SHOOTER_CAM_SETTINGS.name()))
        .withPosition(5, 1)
        .withSize(1, 1);
    autoTab
        .addBoolean("pv 2 good", () -> vision.isConnected(LEFT_INTAKE_CAM_SETTINGS.name()))
        .withPosition(6, 1)
        .withSize(1, 1);
    autoTab
        .addBoolean("pv 3 good", () -> vision.isConnected(RIGHT_INTAKE_CAM_SETTINGS.name()))
        .withPosition(7, 1)
        .withSize(1, 1);
    autoTab
        .addBoolean("pv 4 good", () -> vision.isConnected(RIGHT_SHOOTER_CAM_SETTINGS.name()))
        .withPosition(8, 1)
        .withSize(1, 1);

    autoTab
        .addNumber("ll latency", () -> vision.getLimelightLatency())
        .withPosition(5, 2)
        .withSize(1, 1);

    autoTab
        .addNumber("ll offset", () -> vision.getLimelightTargetOffset().getDegrees())
        .withPosition(6, 2)
        .withSize(1, 1);

    autoTab.addBoolean("ll good", this::llGood).withPosition(7, 2).withSize(1, 1);

    autoTab.addBoolean("prox 1 good", this::prox1Good).withPosition(5, 3).withSize(1, 1);
    autoTab.addBoolean("prox 2 good", this::prox2Good).withPosition(6, 3).withSize(1, 1);
    autoTab.addBoolean("prox 3 good", this::prox3Good).withPosition(7, 3).withSize(1, 1);

    autoTab.addBoolean("GOOD TO GO", this::autoReady).withPosition(9, 0).withSize(3, 3);

    autoTab
        .add("Field", drivetrain.getField())
        .withPosition(2, 1)
        .withSize(3, 2)
        .withProperties(Map.of("robot_width", 0.9144, "robot_length", 0.9144));

    delaySlider =
        autoTab
            .add("Close 4 Delay", closeFourNoteDelay)
            .withPosition(0, 3)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", 0, "max", 10, "Block increment", 0.5))
            .withSize(2, 1)
            .getEntry();

    // Teleop tab stuff

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
        .add("FIELD", drivetrain.getFieldTele())
        .withSize(3, 3)
        .withPosition(8, 1)
        .withProperties(Map.of("robot_width", 0.9144, "robot_length", 0.9144));

    teleTab.addBoolean("HAVE RING", () -> indexer.ringPresent()).withSize(3, 3).withPosition(5, 1);
    teleTab
        .addBoolean("KIND OF INTAK-ED", () -> intake.ringPresent())
        .withSize(3, 1)
        .withPosition(5, 0);

    teleTab
        .addBoolean("pv 1 good", () -> vision.isConnected(LEFT_SHOOTER_CAM_SETTINGS.name()))
        .withPosition(2, 3)
        .withSize(1, 1);
    teleTab
        .addBoolean("pv 2 good", () -> vision.isConnected(LEFT_INTAKE_CAM_SETTINGS.name()))
        .withPosition(3, 3)
        .withSize(1, 1);
    teleTab
        .addBoolean("pv 3 good", () -> vision.isConnected(RIGHT_INTAKE_CAM_SETTINGS.name()))
        .withPosition(4, 2)
        .withSize(1, 1);
    teleTab
        .addBoolean("pv 4 good", () -> vision.isConnected(RIGHT_SHOOTER_CAM_SETTINGS.name()))
        .withPosition(4, 3)
        .withSize(1, 1);

    teleTab
        .addNumber("Match Time", () -> DriverStation.getMatchTime())
        .withWidget("Match Time")
        .withPosition(0, 3)
        .withSize(2, 1)
        .withProperties(Map.of("red_start_time", 10, "yellow_start_time", 20));

    teleTab
        .addBoolean("STRAIGHT LOB", () -> autoLobState == Shooter.State.LOB_STRAIGHT)
        .withPosition(4, 0)
        .withSize(1, 2);

    teleTab
        .addNumber("arm absolute", () -> Math.toDegrees(shooter.getArmAbsoluteAngle()))
        .withPosition(8, 0)
        .withSize(1, 1);

    teleTab
        .addNumber("arm relative", () -> Math.toDegrees(shooter.getArmAngle()))
        .withPosition(9, 0)
        .withSize(1, 1);

    teleTab.addBoolean("shooter good", this::shooterGood).withPosition(10, 0).withSize(1, 1);

    // Test stuff
    testTab
        .add("zero climbers", new InstantCommand(() -> climbers.zero()))
        .withPosition(4, 1)
        .withSize(2, 2);

    testTab
        .addNumber("climber left", () -> climbers.getLeftPos())
        .withPosition(2, 1)
        .withSize(2, 1);

    testTab.add("run left routine", climbers.leftZeroRoutine()).withSize(2, 1).withPosition(2, 2);
    testTab
        .addBoolean("left touch tripped", () -> climbers.isLeftTouchTripped())
        .withSize(2, 1)
        .withPosition(2, 3);

    testTab
        .addNumber("climber right", () -> climbers.getRightPos())
        .withPosition(6, 1)
        .withSize(2, 1);

    testTab.add("run right routine", climbers.rightZeroRoutine()).withSize(2, 1).withPosition(6, 2);
    testTab
        .addBoolean("right touch tripped", () -> climbers.isRightTouchTripped())
        .withSize(2, 1)
        .withPosition(6, 3);

    // Tuning stuff
    tuningHoodAngle =
        tuningTab
            .add("Hood Angle", 20)
            .withSize(2, 1)
            .withPosition(0, 0)
            .withWidget(BuiltInWidgets.kTextView)
            .getEntry();
    tuningFlywheelSpeed =
        tuningTab
            .add("Flywheel Speed", 10)
            .withSize(2, 1)
            .withPosition(2, 0)
            .withWidget(BuiltInWidgets.kTextView)
            .getEntry();

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
    CENTERED_CLIMB,
    AMP,
    AUTO_AMP,
    GROUND_INTAKE,
    HUMAN_PLAYER_INTAKE,
    AUTO_GROUND_INTAKE,
    AUTO_HP_INTAKE,
    TRAP,
    LOB,
    CLEANSE,
    EJECT_INTAKE,
    TEST
  }
}
