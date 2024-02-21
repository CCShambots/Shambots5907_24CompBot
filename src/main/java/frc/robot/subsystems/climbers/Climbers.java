package frc.robot.subsystems.climbers;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.ShamLib.SMF.StateMachine;

public class Climbers extends StateMachine<Climbers.State> {
  private Climber leftClimber;
  private Climber rightClimber;

  public Climbers(
      ClimberIO leftClimberIO,
      ClimberIO rightClimberIO,
      Trigger tuningInc,
      Trigger tuningDec,
      Trigger tuningStop) {
    super("Climbers", State.UNDETERMINED, State.class);

    leftClimber = new Climber("Left Climber", leftClimberIO, tuningInc, tuningDec, tuningStop);
    rightClimber = new Climber("Right Climber", rightClimberIO, tuningInc, tuningDec, tuningStop);

    addChildSubsystem(leftClimber);
    addChildSubsystem(rightClimber);

    registerStateCommands();
    registerTransitions();
  }

  public void zeroLeft() {
    leftClimber.zero();
  }

  public void zeroRight() {
    rightClimber.zero();
  }

  public void zero() {
    zeroLeft();
    zeroRight();
  }

  public double getLeftPos() {
    return leftClimber.getPos();
  }

  public double getRightPos() {
    return rightClimber.getPos();
  }

  private void registerStateCommands() {
    registerStateCommand(
        State.SOFT_E_STOP,
        new ParallelCommandGroup(
                leftClimber.transitionCommand(Climber.State.SOFT_E_STOP),
                rightClimber.transitionCommand(Climber.State.SOFT_E_STOP))
            .andThen(watchSetpointCommand()));

    registerStateCommand(
        State.FREE_EXTEND,
        new ParallelCommandGroup(
                leftClimber.transitionCommand(Climber.State.FREE_EXTEND),
                rightClimber.transitionCommand(Climber.State.FREE_EXTEND))
            .andThen(watchSetpointCommand()));

    registerStateCommand(
        State.FREE_RETRACT,
        new ParallelCommandGroup(
                leftClimber.transitionCommand(Climber.State.FREE_RETRACT),
                rightClimber.transitionCommand(Climber.State.FREE_RETRACT))
            .andThen(watchSetpointCommand()));

    registerStateCommand(
        State.LOADED_RETRACT,
        new ParallelCommandGroup(
                leftClimber.transitionCommand(Climber.State.LOADED_RETRACT),
                rightClimber.transitionCommand(Climber.State.LOADED_RETRACT))
            .andThen(watchSetpointCommand()));

    registerStateCommand(
        State.VOLTAGE_CALC,
        new ParallelCommandGroup(
            leftClimber.transitionCommand(Climber.State.VOLTAGE_CALC),
            rightClimber.transitionCommand(Climber.State.VOLTAGE_CALC)));
  }

  private void registerTransitions() {
    addOmniTransition(State.SOFT_E_STOP);
    addOmniTransition(State.FREE_RETRACT);
    addOmniTransition(State.FREE_EXTEND);
    addOmniTransition(State.LOADED_RETRACT);

    addTransition(State.SOFT_E_STOP, State.VOLTAGE_CALC);
  }

  private Command watchSetpointCommand() {
    return new RunCommand(
        () -> {
          if (leftClimber.isFlag(Climber.State.AT_SETPOINT)
              && rightClimber.isFlag(Climber.State.AT_SETPOINT)) {
            setFlag(State.AT_SETPOINT);
          } else {
            clearFlag(State.AT_SETPOINT);
          }
        });
  }

  public Command leftZeroRoutine() {
    return leftClimber.transitionCommand(Climber.State.AUTOMATIC_ZERO);
  }

  public Command rightZeroRoutine() {
    return rightClimber.transitionCommand(Climber.State.AUTOMATIC_ZERO);
  }

  @Override
  protected void determineSelf() {
    setState(State.SOFT_E_STOP);
  }

  public enum State {
    UNDETERMINED,
    SOFT_E_STOP,
    FREE_EXTEND,
    FREE_RETRACT,
    LOADED_RETRACT,
    VOLTAGE_CALC,

    // flags
    AT_SETPOINT
  }
}
