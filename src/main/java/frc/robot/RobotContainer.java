// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import frc.robot.ShamLib.SMF.StateMachine;
import frc.robot.subsystems.climbers.ClimberIO;
import frc.robot.subsystems.climbers.ClimberIOReal;
import frc.robot.subsystems.climbers.ClimberIOSim;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.indexer.IndexerIO;
import frc.robot.subsystems.indexer.IndexerIOReal;
import frc.robot.subsystems.indexer.IndexerIOSim;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOReal;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.shooter.arm.ArmIO;
import frc.robot.subsystems.shooter.arm.ArmIOReal;
import frc.robot.subsystems.shooter.arm.ArmIOSim;
import frc.robot.subsystems.shooter.flywheel.FlywheelIO;
import frc.robot.subsystems.shooter.flywheel.FlywheelIOReal;
import frc.robot.subsystems.shooter.flywheel.FlywheelIOSim;
import java.util.function.BooleanSupplier;

public class RobotContainer extends StateMachine<RobotContainer.State> {
  /*private final Intake intake;
  private final Shooter shooter;

  private final Indexer indexer;
  private final Vision vision;
  private final Climbers climbers;*/
  private final Drivetrain drivetrain;

  public RobotContainer() {
    super("Robot Container", State.UNDETERMINED, State.class);

    CommandGenericHID hid = new CommandGenericHID(0);

    // actually do bindings :()

    // TODO: Give actual tuning binds
    /*intake =
    new Intake(
        getIntakeIO(() -> false),
        new Trigger(() -> false),
        new Trigger(() -> false),
        new Trigger(() -> false));*/

    /*shooter =
    new Shooter(
        getArmIO(),
        getFlywheelIO(),
        Translation2d::new,
        () -> 0,
        () -> 0,
        new Trigger(() -> false),
        new Trigger(() -> false),
        new Trigger(() -> false));*/

    // TODO: give good sim bindings
    /*indexer =
    new Indexer(
        getIndexerIO(() -> false, () -> false, () -> false),
        new Trigger(() -> false),
        new Trigger(() -> false),
        new Trigger(() -> false));*/

    /*vision = new Vision("limelight", "pv_instance_1");*/

    drivetrain =
        new Drivetrain(() -> hid.getRawAxis(0), () -> -hid.getRawAxis(1), () -> hid.getRawAxis(4));

    /*vision.addVisionUpdateConsumers(drivetrain::addVisionMeasurements);

    climbers = new Climbers(
            getLeftClimberIO(),
            getRightClimberIO(),
            new Trigger(() -> false),
            new Trigger(() -> false),
            new Trigger(() -> false)
    );*/

    addChildSubsystem(drivetrain);
    /*addChildSubsystem(vision);
    addChildSubsystem(intake);
    addChildSubsystem(shooter);
    addChildSubsystem(indexer);
    addChildSubsystem(climbers);*/

    configureBindings(hid);
  }

  private void configureBindings(CommandGenericHID hid) {
    hid.button(1).onTrue(drivetrain.transitionCommand(Drivetrain.State.AUTO_AMP, false));
    hid.button(1)
        .onFalse(drivetrain.transitionCommand(Drivetrain.State.FIELD_ORIENTED_DRIVE, false));
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

  @Override
  protected void onEnable() {
    drivetrain.syncAlliance();
    drivetrain.requestTransition(Drivetrain.State.FIELD_ORIENTED_DRIVE);
  }

  @Override
  protected void determineSelf() {
    // placeholder
    setState(State.SOFT_E_STOP);
  }

  public double getShooterAngle() {
    return /*shooter.getArmAngle();*/ 0;
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
    SOFT_E_STOP // placeholder
  }
}
