// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.ShamLib.SMF.StateMachine;

public class RobotContainer extends StateMachine<RobotContainer.State> {
  private final Pose3d[][] componentPoses = new Pose3d[2][5];

  private final double[][] componentRelativeMotions = new double[2][5];

  public RobotContainer() {
    super("Robot Container", State.UNDETERMINED, State.class);
  }

  @Override
  protected void determineSelf() {
    // placeholder
    setState(State.IDLE);
  }

  // for the get _ angles/poses/extensions, the first entry is actual and the second is target
  private void updateIntakeAngles() {
    // TODO: make this do the thing
    // idx 0 in the pose/motions arrays
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
