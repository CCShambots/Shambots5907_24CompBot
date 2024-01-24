package frc.robot.subsystems.shooter.flywheel;

import static frc.robot.Constants.Flywheel.Settings.*;
import static frc.robot.Constants.doubleEqual;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.ShamLib.SMF.StateMachine;
import frc.robot.ShamLib.motors.tuning.LinearTuningCommand;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Flywheel extends StateMachine<Flywheel.State> {
  private final FlywheelIO io;
  private final FlywheelInputsAutoLogged inputs = new FlywheelInputsAutoLogged();

  private final DoubleSupplier distanceSpeedProvider;

  public Flywheel(
      FlywheelIO io,
      DoubleSupplier distanceSpeedProvider,
      Trigger tuningInc,
      Trigger tuningDec,
      Trigger tuningStop) {
    super("Shooter Flywheel", State.UNDETERMINED, State.class);

    this.distanceSpeedProvider = distanceSpeedProvider;
    this.io = io;

    registerStateCommands(tuningInc, tuningDec, tuningStop);
    registerTransitions();
  }

  private void registerStateCommands(Trigger tuningInc, Trigger tuningDec, Trigger tuningStop) {
    registerStateCommand(
        State.VOLTAGE_CALC,
        new SequentialCommandGroup(
            new LinearTuningCommand(
                tuningStop,
                tuningInc,
                tuningDec,
                io::setVoltage,
                () -> inputs.velocity,
                () -> inputs.voltage,
                VOLTAGE_INCREMENT),
            transitionCommand(State.IDLE)));

    registerStateCommand(
        State.BASE_SHOT_SPIN,
        new ParallelCommandGroup(
            new InstantCommand(() -> io.setFlywheelTarget(BASE_SHOT_VELOCITY)),
            atSpeedCommand(() -> BASE_SHOT_VELOCITY, SPIN_UP_READY_TOLERANCE)));

    registerStateCommand(State.IDLE, io::stop);

    registerStateCommand(
        State.ACTIVE_ADJUST_SPIN,
        new ParallelCommandGroup(
            new RunCommand(() -> io.setFlywheelTarget(distanceSpeedProvider.getAsDouble())),
            atSpeedCommand(distanceSpeedProvider, SPIN_UP_READY_TOLERANCE)));

    registerStateCommand(State.PASS_THROUGH, () -> io.setFlywheelTarget(PASS_THROUGH_SPEED));

    registerStateCommand(State.CHUTE_INTAKE, () -> io.setFlywheelTarget(CHUTE_INTAKE_SPEED));
  }

  private void registerTransitions() {
    // omni everything cause flywheel spinning around isnt gonna hurt anything
    addOmniTransition(State.IDLE);
    addOmniTransition(State.BASE_SHOT_SPIN);
    addOmniTransition(State.ACTIVE_ADJUST_SPIN);
    addOmniTransition(State.PASS_THROUGH);
    addOmniTransition(State.CHUTE_INTAKE);

    addTransition(State.IDLE, State.VOLTAGE_CALC);
  }

  private Command atSpeedCommand(DoubleSupplier speedProvider, double accuracy) {
    return new RunCommand(
        () -> {
          if (doubleEqual(inputs.velocity, speedProvider.getAsDouble(), accuracy)) {
            setFlag(State.AT_SPEED);
          } else {
            clearFlag(State.AT_SPEED);
          }
        });
  }

  @Override
  protected void update() {
    io.updateInputs(inputs);
    Logger.processInputs(getName(), inputs);
  }

  @Override
  protected void determineSelf() {
    // await control from shooter
    io.resetFollower();
    setState(State.IDLE);
  }

  public enum State {
    UNDETERMINED,
    BASE_SHOT_SPIN,
    IDLE,
    ACTIVE_ADJUST_SPIN,
    PASS_THROUGH,
    CHUTE_INTAKE,
    VOLTAGE_CALC,

    // flags
    AT_SPEED
  }
}
