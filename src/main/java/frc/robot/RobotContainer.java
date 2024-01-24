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

  // for the get _ angles/poses/extensions, the first entry is actual and the second is target
  private void updateShooterAngles() {
    // TODO: make this do the thing
    // idx 0 in the pose/motions arrays
  }

  private void updateElevatorExtensions() {
    // TODO: make this do the thing
    // idx 1 in the pose/motions arrays
  }

  private void updateClawAngles() {
    // TODO: make this do the thing
    // idx 2, 3 in the pose/motions arrays
  }

  private void updateShooterPoses() {
    for (int i = 0; i < 2; i++) {
      componentPoses[i][0] = new Pose3d();

      componentPoses[i][0].transformBy(
          new Transform3d(
              Constants.PhysicalConstants.CHASSIS_TO_SHOOTER,
              new Rotation3d(0, componentRelativeMotions[i][0], 0)));
    }
  }

  private void updateElevatorPoses() {
    for (int i = 0; i < 2; i++) {
      componentPoses[i][1] = new Pose3d();

      // move origin of elevator to origin of shooter
      componentPoses[i][1].transformBy(
          new Transform3d(
              new Pose3d(),
              new Pose3d(Constants.PhysicalConstants.SHOOTER_TO_ELEVATOR, new Rotation3d())));

      // rotate elevator to shooter angle
      componentPoses[i][1].transformBy(new Transform3d(new Pose3d(), componentPoses[i][0]));

      // move elevator to it's extension
      componentPoses[i][1].transformBy(
          new Transform3d(
              new Pose3d(),
              new Pose3d(
                  new Translation3d(0, componentRelativeMotions[i][1], 0), new Rotation3d())));
    }
  }

  private void updateClawPoses() {
    for (int i = 0; i < 4; i++) {
      // stupid, but it will give (0, 3) (1, 3) (0, 4) (1, 4) so it gets both claws in both target
      // and actual poses
      // type is target or actual
      // clawidx is for which part of the claw
      int typeIdx = i % 2;
      int clawIdx = (i / 2) + 2;

      componentPoses[typeIdx][clawIdx] = new Pose3d();

      // move origin of claw part to origin of elevator
      componentPoses[typeIdx][clawIdx].transformBy(
          new Transform3d(new Pose3d(), componentPoses[typeIdx][1]));

      // rotate the claw to it's angle
      componentPoses[typeIdx][clawIdx].transformBy(
          new Transform3d(
              new Pose3d(),
              new Pose3d(
                  new Translation3d(),
                  new Rotation3d(0, componentRelativeMotions[typeIdx][clawIdx], 0))));
    }
  }

  private void updateAllRelative() {
    updateShooterAngles();
    updateElevatorExtensions();
    updateClawAngles();
  }

  private void updateAllPoses() {
    updateShooterPoses();
    updateElevatorPoses();
    updateClawPoses();
  }

  // ik this is kinda stupid, i dont really care
  // im doing it with the arrays n stuff instead of returning to avoid extra allocations cause loop
  // overruns (yippee)
  public Pose3d[][] getComponentPositions() {
    updateAllRelative();
    updateAllPoses();

    return componentPoses;
  }

  public enum State {
    UNDETERMINED,
    SOFT_E_STOP // placeholder
  }
}
