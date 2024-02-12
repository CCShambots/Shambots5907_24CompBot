// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.ShamLib.HID.CommandFlightStick;
import frc.robot.ShamLib.SMF.StateMachine;
import frc.robot.commands.DetermineRingStatusCommand;
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
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import frc.robot.util.StageSide;
import java.util.function.BooleanSupplier;

public class RobotContainer extends StateMachine<RobotContainer.State> {
  private final Intake intake;
  private final Shooter shooter;
  private final Indexer indexer;
  private final Vision vision;
  private final Climbers climbers;
  private final Drivetrain drivetrain;
  private final Lights lights;

  private final CommandXboxController operatorController = new CommandXboxController(Constants.Controller.OPERATOR_CONTROLLER_ID);
  private final CommandFlightStick leftFlightStick = new CommandFlightStick(Constants.Controller.LEFT_FLIGHT_STICK_ID);
  private final CommandFlightStick rightFlightStick = new CommandFlightStick(Constants.Controller.RIGHT_FLIGHT_STICK_ID);

  private final LoggedDashboardChooser<Command> autoChooser;

  private StageSide targetStageSide = StageSide.CENTER;

  public RobotContainer(EventLoop checkModulesLoop) {
    super("Robot Container", State.UNDETERMINED, State.class);

    autoChooser = new LoggedDashboardChooser<>("Logged Autonomous Chooser", AutoBuilder.buildAutoChooser());

    // actually do bindings :()

    // TODO: Give actual tuning binds
    intake =
        new Intake(
            getIntakeIO(operatorController.povDown()),
            tuningIncrement(),
            tuningDecrement(),
            tuningStop());

    // TODO: give good sim bindings
    indexer =
        new Indexer(
            getIndexerIO(
                    operatorController.povLeft(),
                    operatorController.povUp(),
                    operatorController.povRight()),
            tuningIncrement(),
            tuningDecrement(),
            tuningStop());

    vision = new Vision("limelight", "pv_instance_1");

    drivetrain =
        new Drivetrain(
            () -> -leftFlightStick.getY(),
            () -> -leftFlightStick.getX(),
            () -> -rightFlightStick.getRawAxis(0),
            () -> targetStageSide,
            tuningIncrement(),
            tuningDecrement(),
            tuningStop());

    drivetrain.registerMisalignedSwerveTriggers(checkModulesLoop);

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
    addChildSubsystem(lights);

    registerStateCommands();
    registerTransitions();

    configureBindings();
  }

  private void registerStateCommands() {
    registerStateCommand(State.SOFT_E_STOP, new ParallelCommandGroup(
            drivetrain.transitionCommand(Drivetrain.State.IDLE),
            climbers.transitionCommand(Climbers.State.SOFT_E_STOP),
            intake.transitionCommand(Intake.State.IDLE),
            shooter.transitionCommand(Shooter.State.SOFT_E_STOP),
            indexer.transitionCommand(Indexer.State.IDLE),
            lights.transitionCommand(Lights.State.ERROR)
    ));

    registerStateCommand(State.TRAVERSING, new ParallelCommandGroup(
            drivetrain.transitionCommand(Drivetrain.State.FIELD_ORIENTED_DRIVE),
            climbers.transitionCommand(Climbers.State.FREE_RETRACT),
            intake.transitionCommand(Intake.State.IDLE),
            new DetermineRingStatusCommand(shooter, indexer, lights)
    ));


  }

  private void registerTransitions() {
    addOmniTransition(State.TRAVERSING);
    addOmniTransition(State.SOFT_E_STOP);
  }

  private Trigger tuningIncrement() {
    return operatorController.povUp();
  }

  private Trigger tuningDecrement() {
    return operatorController.povDown();
  }

  private Trigger tuningStop() {
    return operatorController.a();
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
    // placeholder
    setState(State.SOFT_E_STOP);
  }

  public double getShooterAngle() {
    return shooter.getArmAngle();
  }

  public Pose3d getBotPose() {
    // update this when pose estimation is ready
    Pose2d pose = drivetrain.getBotPose();
    return new Pose3d(
        new Translation3d(pose.getX(), pose.getY(), 0),
        new Rotation3d(0, 0, pose.getRotation().getRadians()));
  }

  public enum State {
    UNDETERMINED,
    AUTONOMOUS,
    TRAVERSING,

    SOFT_E_STOP
  }
}
