// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.ShamLib.AllianceManager;
import frc.robot.ShamLib.SMF.StateMachine;
import frc.robot.ShamLib.ShamLibConstants;
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
import frc.robot.subsystems.lights.LightsIOSim;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.arm.ArmIO;
import frc.robot.subsystems.shooter.arm.ArmIOReal;
import frc.robot.subsystems.shooter.arm.ArmIOSim;
import frc.robot.subsystems.shooter.flywheel.FlywheelIO;
import frc.robot.subsystems.shooter.flywheel.FlywheelIOReal;
import frc.robot.subsystems.shooter.flywheel.FlywheelIOSim;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.StageSide;
import java.util.function.BooleanSupplier;
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

  private final CommandGenericHID triggerSims = new CommandGenericHID(1);

  private final LoggedDashboardChooser<Command> autoChooser;

  private StageSide targetStageSide = StageSide.CENTER;

  public RobotContainer(EventLoop checkModulesLoop) {
    super("Robot Container", State.UNDETERMINED, State.class);

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

    // vision = new Vision("limelight", "pv_instance_1");
    vision = new Vision("limelight");

    drivetrain =
        new Drivetrain(
            controllerBindings::getDriveXValue,
            controllerBindings::getDriveYValue,
            controllerBindings::getDriveTurnValue,
            () -> targetStageSide,
            tuningIncrement(),
            tuningDecrement(),
            tuningStop());

    drivetrain.registerMisalignedSwerveTriggers(checkModulesLoop);

    // vision.addVisionUpdateConsumers(drivetrain::addVisionMeasurements);

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
    // addChildSubsystem(vision);
    addChildSubsystem(intake);
    addChildSubsystem(shooter);
    addChildSubsystem(indexer);
    addChildSubsystem(climbers);
    addChildSubsystem(lights);

    registerStateCommands();
    registerTransitions();

    configureBindings();

    // Important to instatiate after drivetrain consructor is called so that auto builder is
    // configured
    autoChooser =
        new LoggedDashboardChooser<>("Logged Autonomous Chooser", AutoBuilder.buildAutoChooser());

    initializeDriveTab();
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
            shooter.transitionCommand(Shooter.State.PARTIAL_STOW),
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
  }

  private void registerTransitions() {
    addOmniTransition(State.TRAVERSING);
    addOmniTransition(State.SOFT_E_STOP);
    addOmniTransition(State.SPEAKER_SCORE);
    addOmniTransition(State.BASE_SHOT);
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

  private void configureBindings() {}

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

      case SIM -> {
        return new LightsIOSim();
      }

      default -> {
        return new LightsIO() {};
      }
    }
  }

  @Override
  protected void onEnable() {}

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

  private void initializeDriveTab() {
    ShuffleboardTab driveTab = Shuffleboard.getTab("Drive");

    driveTab.add("Auto Route", autoChooser.getSendableChooser()).withPosition(3, 0).withSize(2, 2);

    driveTab
        .addString("ALLIANCE", () -> AllianceManager.getAlliance().name())
        .withPosition(0, 0)
        .withSize(2, 2);
    driveTab
        .add("SWITCH ALLIANCE", AllianceManager.switchAlliance())
        .withPosition(5, 2)
        .withSize(2, 2);
    driveTab.add("SYNC ALLIANCE", AllianceManager.syncAlliance()).withPosition(5, 0).withSize(2, 2);

    driveTab
        .addNumber("arm absolute", () -> Math.toDegrees(shooter.getArmAbsoluteAngle()))
        .withPosition(1, 2)
        .withSize(1, 1);

    driveTab
        .addNumber("arm relative", () -> Math.toDegrees(shooter.getArmAngle()))
        .withPosition(0, 2)
        .withSize(1, 1);
  }

  public enum State {
    UNDETERMINED,
    AUTONOMOUS,
    TRAVERSING,
    SPEAKER_SCORE,
    GROUND_INTAKE,
    BASE_SHOT,
    SOFT_E_STOP
  }
}
