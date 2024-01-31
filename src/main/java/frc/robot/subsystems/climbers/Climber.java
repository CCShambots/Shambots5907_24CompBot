package frc.robot.subsystems.climbers;

import static frc.robot.Constants.Climbers.Settings.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.ShamLib.SMF.StateMachine;
import org.littletonrobotics.junction.Logger;

public class Climber extends StateMachine<Climber.State> {
  private final ClimberIO io;
  private final ClimberInputsAutoLogged inputs = new ClimberInputsAutoLogged();

  public Climber(String name, ClimberIO io) {
    super(name, State.UNDETERMINED, State.class);

    this.io = io;

    registerStateCommands();
    registerTransitions();
  }

  private void registerStateCommands() {
    registerStateCommand(State.FREE_EXTEND, new SequentialCommandGroup(
            new InstantCommand(() -> {
              io.setSpeed(FREE_VELOCITY, FREE_ACCELERATION, FREE_JERK);
              io.setControlSlot(FREE_SLOT);
              io.setTarget(EXTENSION_SETPOINT);
            }),
            watchSetpointCommand()
    ));

    registerStateCommand(State.FREE_RETRACT, new SequentialCommandGroup(
            new InstantCommand(() -> {
              io.setSpeed(FREE_VELOCITY, FREE_ACCELERATION, FREE_JERK);
              io.setControlSlot(FREE_SLOT);
              io.setTarget(0);
            }),
            watchSetpointCommand()
    ));

    registerStateCommand(State.LOADED_RETRACT, new SequentialCommandGroup(
            new InstantCommand(() -> {
              io.setSpeed(LOADED_VELOCITY, LOADED_ACCELERATION, LOADED_JERK);
              io.setControlSlot(LOADED_SLOT);
              io.setTarget(0);
            }),
            watchSetpointCommand()
    ));

    registerStateCommand(State.SOFT_E_STOP, io::stop);
  }

  private void registerTransitions() {
    addOmniTransition(State.SOFT_E_STOP);
    addOmniTransition(State.FREE_RETRACT);
    addOmniTransition(State.FREE_EXTEND);
    addOmniTransition(State.LOADED_RETRACT);
  }

  private Command watchSetpointCommand() {
    return new RunCommand(() -> {
      if (Constants.doubleEqual(inputs.position, inputs.targetPosition, SETPOINT_TOLERANCE)) {
        setFlag(State.AT_SETPOINT);
      }
      else {
        clearFlag(State.AT_SETPOINT);
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
    setState(State.SOFT_E_STOP);
  }

  public enum State {
    UNDETERMINED,
    FREE_EXTEND,
    FREE_RETRACT,
    LOADED_RETRACT,
    SOFT_E_STOP,

    //flags
    AT_SETPOINT
  }
}
