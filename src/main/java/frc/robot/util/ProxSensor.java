package frc.robot.util;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ProxSensor implements Sendable {
  private final DigitalInput input;

  public ProxSensor(int pin) {
    input = new DigitalInput(pin);
  }

  public boolean isActivated() {
    return !input.get();
  }

  /**
   * Create a new trigger to run when the state of the prox sensor changes to 'value'
   *
   * @param value When the prox sensor reaches this value, it will run the runnable
   * @param toRun What to run
   */
  public void registerTrigger(boolean value, Runnable toRun) {
    new Trigger(() -> this.isActivated() == value).onTrue(new InstantCommand(toRun));
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("prox-sensor");
    builder.addBooleanProperty("activated", () -> isActivated(), null);
  }
}
