package frc.robot.controllers;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface ControllerBindings {

  double getDriveXValue();

  double getDriveYValue();

  double getDriveTurnValue();

  default Trigger enterTune() {
    return new Trigger(() -> false);
  }
  ;

  Trigger tuningIncrement();

  Trigger tuningDecrement();

  Trigger tuningStop();

  Trigger feedOnPress();

  Trigger resetGyro();

  Trigger groundIntake();

  Trigger manualGroundIntake();

  Trigger shoot();

  Trigger indicateNonSourceNote();

  Trigger indicateSourceNote();

  Trigger manualBaseShot();

  Trigger humanPlayerIntake();

  Trigger traversing();

  Trigger xShape();

  Trigger autoAmp();

  Trigger lobShot();

  Trigger manualAmp();

  Trigger trapScore();

  default Trigger toggleLobMode() {
    return new Trigger(() -> false);
  }

  default Trigger resetVisionPose() {
    return new Trigger(() -> false);
  }

  default Trigger preExtendClimbers() {
    return new Trigger(() -> false);
  }

  Trigger startClimb();

  Trigger retractClimb();

  Trigger cleanse();

  Trigger ejectIntake();

  Trigger targetLeftStage();

  Trigger targetRightStage();

  Trigger targetCenterStage();

  // Simulated prox sensors for running in sim mode
  default Trigger simProx1() {
    return new Trigger(() -> false);
  }

  default Trigger simProx2() {
    return new Trigger(() -> false);
  }

  default Trigger simProx3() {
    return new Trigger(() -> false);
  }

  default Trigger simProx4() {
    return new Trigger(() -> false);
  }

  default void setRumble(double rumbleValue) {}
}
