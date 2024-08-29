package frc.robot.controllers;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.ShamLib.HID.CommandFlightStick;

public class RealControllerBindings implements ControllerBindings {

  private final CommandXboxController operatorController =
      new CommandXboxController(Constants.Controller.OPERATOR_CONTROLLER_ID);
  private final CommandFlightStick leftFlightStick =
      new CommandFlightStick(Constants.Controller.LEFT_FLIGHT_STICK_ID);
  private final CommandFlightStick rightFlightStick =
      new CommandFlightStick(Constants.Controller.RIGHT_FLIGHT_STICK_ID);

  @Override
  public double getDriveXValue() {
    return -rightFlightStick.getY();
  }

  @Override
  public double getDriveYValue() {
    return -rightFlightStick.getX();
  }

  @Override
  public double getDriveTurnValue() {
    return -leftFlightStick.getRawAxis(0);
  }

  @Override
  public Trigger feedOnPress() {
    return operatorController.rightBumper();
  }

  @Override
  public Trigger shoot() {
    return rightFlightStick.trigger();
  }

  @Override
  public Trigger manualBaseShot() {
    return operatorController.b();
  }

  @Override
  public Trigger groundIntake() {
    return leftFlightStick.trigger();
  }

  @Override
  public Trigger manualGroundIntake() {
    return operatorController.a();
  }

  @Override
  public Trigger humanPlayerIntake() {
    return operatorController.x();
  }

  @Override
  public Trigger traversing() {
    return operatorController.leftBumper();
  }

  @Override
  public Trigger lobShot() {
    return operatorController.leftTrigger();
  }

  @Override
  public Trigger toggleLobMode() {
    return operatorController.pov(180);
  }

  @Override
  public Trigger autoAmp() {
    return rightFlightStick.pov(0).or(rightFlightStick.pov(45)).or(rightFlightStick.pov(315));
  }

  @Override
  public Trigger manualAmp() {
    return operatorController.y();
  }

  @Override
  public Trigger indicateNonSourceNote() {
    return new Trigger(() -> operatorController.getLeftY() < -0.25);
  }

  @Override
  public Trigger indicateSourceNote() {
    return new Trigger(() -> operatorController.getLeftY() > 0.25);
  }

  @Override
  public Trigger resetVisionPose() {
    return rightFlightStick.topRight();
  }

  @Override
  public Trigger trapScore() {
    return leftFlightStick.pov(0).or(leftFlightStick.pov(45)).or(leftFlightStick.pov(315));
  }

  @Override
  public Trigger targetCenterStage() {
    return operatorController.povUp();
  }

  @Override
  public Trigger targetLeftStage() {
    return operatorController.povLeft();
  }

  @Override
  public Trigger targetRightStage() {
    return operatorController.povRight();
  }

  @Override
  public Trigger retractClimb() {
    return operatorController.rightTrigger(0.5);
  }

  @Override
  public Trigger preExtendClimbers() {
    return operatorController.rightStick();
  }

  @Override
  public Trigger startClimb() {
    return leftFlightStick.topBase();
  }

  @Override
  public Trigger cleanse() {
    return operatorController.button(7);
  }

  @Override
  public Trigger ejectIntake() {
    return operatorController.button(8);
  }

  @Override
  public Trigger xShape() {
    return rightFlightStick.topBase();
  }

  @Override
  public Trigger tuningIncrement() {
    return operatorController.povUp();
  }

  @Override
  public Trigger tuningDecrement() {
    return operatorController.povDown();
  }

  @Override
  public Trigger tuningStop() {
    return operatorController.a();
  }

  @Override
  public Trigger resetGyro() {
    return rightFlightStick.topLeft();
  }

  @Override
  public void setRumble(double rumbleValue) {
    operatorController.getHID().setRumble(RumbleType.kBothRumble, rumbleValue);
  }

  @Override
  public Trigger enterTune() {
    return rightFlightStick.button(13);
  }
}
