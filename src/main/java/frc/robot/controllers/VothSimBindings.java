package frc.robot.controllers;

import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class VothSimBindings implements ControllerBindings {
  private final CommandGenericHID leftKeyboard = new CommandGenericHID(0);
  private final CommandGenericHID rightKeyboard = new CommandGenericHID(1);

  @Override
  public Trigger simProx1() {
    return leftKeyboard.button(1);
  }

  @Override
  public Trigger simProx2() {
    return leftKeyboard.button(2);
  }

  @Override
  public Trigger simProx3() {
    return leftKeyboard.button(3);
  }

  @Override
  public Trigger simProx4() {
    return leftKeyboard.button(4);
  }

  @Override
  public double getDriveXValue() {
    return leftKeyboard.getRawAxis(1);
  }

  @Override
  public double getDriveYValue() {
    return leftKeyboard.getRawAxis(2);
  }

  @Override
  public Trigger lobShot() {
    return new Trigger(() -> false);
  }

  @Override
  public double getDriveTurnValue() {
    return rightKeyboard.getRawAxis(1);
  }

  @Override
  public Trigger tuningIncrement() {
    return new Trigger(() -> false);
  }

  @Override
  public Trigger tuningDecrement() {
    return new Trigger(() -> false);
  }

  @Override
  public Trigger tuningStop() {
    return new Trigger(() -> true);
  }

  @Override
  public Trigger feedOnPress() {
    return new Trigger(() -> true);
  }

  @Override
  public Trigger resetGyro() {
    return new Trigger(() -> false);
  }

  @Override
  public Trigger groundIntake() {
    return new Trigger(() -> false);
  }

  @Override
  public Trigger manualGroundIntake() {
    return rightKeyboard.button(1);
  }

  @Override
  public Trigger shoot() {
    return rightKeyboard.button(2);
  }

  @Override
  public Trigger indicateNonSourceNote() {
    return new Trigger(() -> false);
  }

  @Override
  public Trigger indicateSourceNote() {
    return new Trigger(() -> false);
  }

  @Override
  public Trigger manualBaseShot() {
    return new Trigger(() -> false);
  }

  @Override
  public Trigger humanPlayerIntake() {
    return new Trigger(() -> false);
  }

  @Override
  public Trigger traversing() {
    return rightKeyboard.button(2);
  }

  @Override
  public Trigger xShape() {
    return new Trigger(() -> false);
  }

  @Override
  public Trigger autoAmp() {
    return new Trigger(() -> false);
  }

  @Override
  public Trigger manualAmp() {
    return new Trigger(() -> false);
  }

  @Override
  public Trigger trapScore() {
    return rightKeyboard.button(4);
  }

  @Override
  public Trigger startClimb() {
    return new Trigger(() -> false);
  }

  @Override
  public Trigger retractClimb() {
    return new Trigger(() -> false);
  }

  @Override
  public Trigger cleanse() {
    return rightKeyboard.button(3);
  }

  @Override
  public Trigger ejectIntake() {
    return new Trigger(() -> false);
  }

  @Override
  public Trigger targetLeftStage() {
    return new Trigger(() -> false);
  }

  @Override
  public Trigger targetRightStage() {
    return new Trigger(() -> false);
  }

  @Override
  public Trigger targetCenterStage() {
    return new Trigger(() -> false);
  }
}
