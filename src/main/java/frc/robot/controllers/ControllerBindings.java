package frc.robot.controllers;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface ControllerBindings {

  double getDriveXValue();

  double getDriveYValue();

  double getDriveTurnValue();

  Trigger tuningIncrement();

  Trigger tuningDecrement();

  Trigger tuningStop();

  Trigger feedOnPress();

  // Simulated prox sensors for running in sim mode
  default Trigger simProx1() {
    return new Trigger(() -> false);
  }

  default Trigger simProx2() {
    return new Trigger(() -> false);
  }
  ;

  default Trigger simProx3() {
    return new Trigger(() -> false);
  }
  ;

  default Trigger simProx4() {
    return new Trigger(() -> false);
  }
  ;
}
