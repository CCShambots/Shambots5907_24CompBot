package frc.robot.controllers;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class SimControllerBindings implements ControllerBindings {
  private final CommandXboxController operatorController = new CommandXboxController(0);

  public SimControllerBindings() {}

  @Override
  public double getDriveXValue() {
    return -operatorController.getLeftY();
  }

  @Override
  public double getDriveYValue() {
    return -operatorController.getLeftX();
  }

  @Override
  public double getDriveTurnValue() {
    return -operatorController.getRightX();
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
  public Trigger humanPlayerIntake() {
    return operatorController.x();
  }

  @Override
  public Trigger traversing() {
    return operatorController.leftBumper();
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
  public Trigger feedOnPress() {
    return operatorController.a();
  }

  @Override
  public Trigger resetGyro() {
    return operatorController.b();
  }

  @Override
  public Trigger simProx1() {
    return operatorController.povDown();
  }

  @Override
  public Trigger simProx2() {
    return operatorController.a();
  }

  @Override
  public Trigger simProx3() {
    return operatorController.b();
  }

  @Override
  public Trigger simProx4() {
    return operatorController.x();
  }
}
