package frc.robot.controllers;

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
    return -leftFlightStick.getY();
  }

  @Override
  public double getDriveYValue() {
    return -leftFlightStick.getX();
  }

  @Override
  public double getDriveTurnValue() {
    return -rightFlightStick.getRawAxis(0);
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
  public Trigger autoAmp() {
    return rightFlightStick.pov(0).or(rightFlightStick.pov(45)).or(rightFlightStick.pov(315));
  }

  @Override
  public Trigger indicateAmpIntention() {
    return operatorController.y();
  }

  @Override
  public Trigger trapScore() {
    return operatorController.leftTrigger(0.5);
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
  public Trigger startClimb() {
    return leftFlightStick.topBase();
  }

  @Override
  public Trigger cleanse() {
    return operatorController.button(7).and(operatorController.button(8));
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
    return leftFlightStick.topLeft();
  }
}
