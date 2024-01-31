package frc.robot.subsystems.climbers;

import static frc.robot.Constants.Climbers.Settings.*;

import edu.wpi.first.wpilibj2.command.Command;
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
    registerStateCommand(State.FREE_EXTEND, () -> {
      io.setSpeed(FREE_VELOCITY, FREE_ACCELERATION, FREE_JERK);
    });
  }

  private void registerTransitions() {

  }

  private Command watchSetpointCommand() {

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
