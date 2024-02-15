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
  public Trigger baseShot() {
    return operatorController.b();
  }

  @Override
  public Trigger groundIntake() {
    return operatorController.a();
  }

  @Override
  public Trigger hpIntake() {
    return operatorController.x();
  }

  @Override
  public Trigger traversing() {
    return operatorController.leftBumper();
  }

  @Override
  public Trigger xShape() {
    return rightFlightStick.trigger();
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
    return rightFlightStick.topBase();
  }
}
