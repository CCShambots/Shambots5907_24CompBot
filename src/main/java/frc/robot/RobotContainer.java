// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.ShamLib.SMF.StateMachine;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOReal;
import frc.robot.subsystems.intake.IntakeIOSim;

public class RobotContainer extends StateMachine<RobotContainer.State> {
  private final Pose3d[][] componentPoses = new Pose3d[2][5];

  private final double[][] componentRelativeMotions = new double[2][5];

  private final Intake intake;

  public RobotContainer() {
    super("Robot Container", State.UNDETERMINED, State.class);

    // TODO: make this not just ignore manual control :()
    intake = new Intake(getIntakeIO(), new Trigger(() -> false));

    addChildSubsystem(intake);

    SmartDashboard.putData("intake", intake);

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

  @Override
  protected void onEnable() {
    intake.requestTransition(Intake.State.SEEKING_STOWED_NO_EXPEL);
  }

  @Override
  protected void determineSelf() {
    // placeholder
    setState(State.IDLE);
  }

  // for the get _ angles/poses/extensions, the first entry is actual and the second is target
  private void updateIntakeAngles() {
    // idx 0 in the pose/motions arrays
    componentRelativeMotions[0][0] = intake.getArmPosition();
    componentRelativeMotions[1][0] = intake.getArmTargetPosition();
  }

  private void updateShooterAngles() {
    // TODO: make this do the thing
    // idx 1 in the pose/motions arrays
  }

  private void updateElevatorExtensions() {
    // TODO: make this do the thing
    // idx 2 in the pose/motions arrays
  }

  private void getClawAngles() {
    // TODO: make this do the thing
    // idx 3, 4 in the pose/motions arrays
  }

  private void updateIntakePoses() {
    for (int i = 0; i < 2; i++) {
      componentPoses[i][0] = new Pose3d();

      componentPoses[i][0].transformBy(
          new Transform3d(
              Constants.PhysicalConstants.CHASSIS_TO_INTAKE,
              new Rotation3d(0, componentRelativeMotions[i][0], 0)));
    }
  }

  private void updateShooterPoses() {
    for (int i = 0; i < 2; i++) {
      componentPoses[i][1] = new Pose3d();

      componentPoses[i][1].transformBy(
          new Transform3d(
              Constants.PhysicalConstants.CHASSIS_TO_SHOOTER,
              new Rotation3d(0, componentRelativeMotions[i][1], 0)));
    }
  }

  private void updateElevatorPoses() {
    for (int i = 0; i < 2; i++) {
      componentPoses[i][2] = new Pose3d();

      // move origin of elevator to origin of shooter
      componentPoses[i][2].transformBy(
          new Transform3d(
              new Pose3d(),
              new Pose3d(Constants.PhysicalConstants.SHOOTER_TO_ELEVATOR, new Rotation3d())));

      // rotate elevator to shooter angle
      componentPoses[i][2].transformBy(new Transform3d(new Pose3d(), componentPoses[i][1]));

      // move elevator to it's extension
      componentPoses[i][2].transformBy(
          new Transform3d(
              new Pose3d(),
              new Pose3d(
                  new Translation3d(0, componentRelativeMotions[i][2], 0), new Rotation3d())));
    }
  }

  private void updateClawPoses() {
    for (int i = 0; i < 4; i++) {
      // stupid, but it will give (0, 3) (1, 3) (0, 4) (1, 4) so it gets both claws in both target
      // and actual poses
      // type is target or actual
      // clawidx is for which part of the claw
      int typeIdx = i % 2;
      int clawIdx = (i / 2) + 3;

      componentPoses[typeIdx][clawIdx] = new Pose3d();

      // move origin of claw part to origin of elevator
      componentPoses[typeIdx][clawIdx].transformBy(
          new Transform3d(new Pose3d(), componentPoses[typeIdx][2]));

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
    updateIntakeAngles();
    updateShooterAngles();
    updateElevatorExtensions();
    updateClawPoses();
  }

  private void updateAllPoses() {
    updateIntakePoses();
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
    IDLE // placeholder
  }
}
