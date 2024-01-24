// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.ShamLib.SMF.StateMachine;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerIO;
import frc.robot.subsystems.indexer.IndexerIOReal;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOReal;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.arm.ArmIO;
import frc.robot.subsystems.shooter.arm.ArmIOReal;
import frc.robot.subsystems.shooter.arm.ArmIOSim;
import frc.robot.subsystems.shooter.flywheel.FlywheelIO;
import frc.robot.subsystems.shooter.flywheel.FlywheelIOReal;
import frc.robot.subsystems.shooter.flywheel.FlywheelIOSim;

public class RobotContainer extends StateMachine<RobotContainer.State> {
  private final Pose3d[][] componentPoses = new Pose3d[2][4];

  private final double[][] componentRelativeMotions = new double[2][4];

  private final Intake intake;
  private final Shooter shooter;

  private final Indexer indexer;

  public RobotContainer() {
    super("Robot Container", State.UNDETERMINED, State.class);

    // TODO: Give actual tuning binds
    intake =
        new Intake(
            getIntakeIO(),
            new Trigger(() -> false),
            new Trigger(() -> false),
            new Trigger(() -> false));

    shooter =
        new Shooter(
            getArmIO(),
            getFlywheelIO(),
            Translation3d::new,
            () -> 0,
            () -> 0,
            new Trigger(() -> false),
            new Trigger(() -> false),
            new Trigger(() -> false));

    indexer =
        new Indexer(
            getIndexerIO(),
            new Trigger(() -> false),
            new Trigger(() -> false),
            new Trigger(() -> false));

    addChildSubsystem(intake);
    addChildSubsystem(shooter);
    addChildSubsystem(indexer);

    configureBindings();
  }

  private void configureBindings() {}

  private IntakeIO getIntakeIO() {
    switch (Constants.currentBuildMode) {
      case REAL -> {
        return new IntakeIOReal();
      }
      case SIM -> {
        return new IntakeIOSim();
      }
      default -> {
        return new IntakeIO() {};
      }
    }
  }

  private final IndexerIO getIndexerIO() {
    switch (Constants.currentBuildMode) {
      case REAL -> {
        return new IndexerIOReal();
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
    //update this when pose estimation is ready
    return new Pose3d();
  }

  public enum State {
    UNDETERMINED,
    SOFT_E_STOP // placeholder
  }
}
