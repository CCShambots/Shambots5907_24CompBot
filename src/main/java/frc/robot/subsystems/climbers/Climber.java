package frc.robot.subsystems.climbers;

import static frc.robot.Constants.Climbers.Settings.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.ShamLib.SMF.StateMachine;
import frc.robot.ShamLib.motors.tuning.LinearTuningCommand;
import org.littletonrobotics.junction.Logger;

public class Climber extends StateMachine<Climber.State> {
  private final ClimberIO io;
  private final ClimberInputsAutoLogged inputs = new ClimberInputsAutoLogged();

  public Climber(
      String name, ClimberIO io, Trigger tuningInc, Trigger tuningDec, Trigger tuningStop) {
    super(name, State.UNDETERMINED, State.class);

    this.io = io;

    registerStateCommands(tuningInc, tuningDec, tuningStop);
    registerTransitions();
  }

  public void zero() {
    io.resetPosition();
  }

  private void registerStateCommands(Trigger tuningInc, Trigger tuningDec, Trigger tuningStop) {
    registerStateCommand(
        State.FREE_EXTEND,
        new SequentialCommandGroup(
            new InstantCommand(
                () -> {
                  io.setSpeed(FREE_VELOCITY, FREE_ACCELERATION, FREE_JERK);
                  io.setControlSlot(FREE_SLOT);
                  io.setTarget(EXTENSION_SETPOINT);
                }),
            watchSetpointCommand()));

    registerStateCommand(
        State.MINIMUM_EXTEND,
        new SequentialCommandGroup(
            new InstantCommand(
                () -> {
                  io.setSpeed(FREE_VELOCITY, FREE_ACCELERATION, FREE_JERK);
                  io.setControlSlot(FREE_SLOT);
                  io.setTarget(MINIMUM_EXTENSION_SETPOINT);
                }),
            watchSetpointCommand()));

    registerStateCommand(
        State.FREE_RETRACT,
        new SequentialCommandGroup(
            new InstantCommand(
                () -> {
                  io.setSpeed(FREE_VELOCITY, FREE_ACCELERATION, FREE_JERK);
                  io.setControlSlot(FREE_SLOT);
                  io.setTarget(0);
                }),
            watchSetpointCommand()));

    registerStateCommand(
        State.LOADED_RETRACT,
        new SequentialCommandGroup(
            new InstantCommand(
                () -> {
                  io.setSpeed(LOADED_VELOCITY, LOADED_ACCELERATION, LOADED_JERK);
                  io.setControlSlot(LOADED_SLOT);
                  io.setTarget(RETRACT_SETPOINT);
                }),
            watchSetpointCommand()));

    registerStateCommand(State.SOFT_E_STOP, io::stop);

    registerStateCommand(
        State.VOLTAGE_CALC,
        new LinearTuningCommand(
            tuningStop,
            tuningInc,
            tuningDec,
            io::setVoltage,
            () -> inputs.rotorVelocity,
            () -> inputs.voltage,
            VOLTAGE_INCREMENT));

    registerStateCommand(State.AUTOMATIC_ZERO, new AutomaticZeroCommand(this, io, AUTO_ZERO_POWER));
  }

  private void registerTransitions() {
    addOmniTransition(State.SOFT_E_STOP);
    addOmniTransition(State.FREE_RETRACT);
    addOmniTransition(State.FREE_EXTEND);
    addOmniTransition(State.MINIMUM_EXTEND);
    addOmniTransition(State.LOADED_RETRACT);

    addTransition(State.SOFT_E_STOP, State.AUTOMATIC_ZERO);

    addTransition(State.SOFT_E_STOP, State.VOLTAGE_CALC);
  }

  private Command watchSetpointCommand() {
    return new RunCommand(
        () -> {
          if (Constants.doubleEqual(inputs.position, inputs.targetPosition, SETPOINT_TOLERANCE)) {
            setFlag(State.AT_SETPOINT);
          } else {
            clearFlag(State.AT_SETPOINT);
          }
        });
  }

  public double getClimberVelo() {
    return inputs.velocity;
  }

  @Override
  protected void update() {
    io.updateInputs(inputs);
    Logger.processInputs(getName(), inputs);
  }

  @Override
  protected void determineSelf() {
    setState(State.SOFT_E_STOP);
  }

  public double getPos() {
    return inputs.position;
  }

  public boolean isTouchTripped() {
    return inputs.touchTripped;
  }

  public enum State {
    UNDETERMINED,
    FREE_EXTEND,
    MINIMUM_EXTEND,
    FREE_RETRACT,
    LOADED_RETRACT,
    SOFT_E_STOP,
    VOLTAGE_CALC,

    AUTOMATIC_ZERO,

    // flags
    AT_SETPOINT
  }
}
