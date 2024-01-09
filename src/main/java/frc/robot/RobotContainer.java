// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.ShamLib.SMF.StateMachine;

public class RobotContainer extends StateMachine<RobotContainer.State> {
  public RobotContainer() {
      super("Robot Container", State.UNDETERMINED, State.class);
  }

  @Override
  protected void determineSelf() {
    //placeholder
    setState(State.IDLE);
  }

  public enum State {
    UNDETERMINED,
    IDLE //placeholder
  }
}
