package frc.robot.subsystems.lights;

import static frc.robot.Constants.Lights.Settings.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ShamLib.SMF.StateMachine;
import frc.robot.ShamLib.WhileDisabledInstantCommand;

public class Lights extends StateMachine<Lights.State> {
  private final LightsIO io;
  private final LightsInputsAutoLogged inputs = new LightsInputsAutoLogged();

  public Lights(LightsIO io) {
    super("Lights", State.UNDETERMINED, State.class);

    this.io = io;

    registerStateCommmands();
    registerTransitions();
  }

  private void registerTransitions() {
    addOmniTransitions(State.values());
  }

  private void registerStateCommmands() {
    registerStateCommand(State.RESTING, setLights(State.RESTING));
    registerStateCommand(State.AUTO, setLights(State.AUTO));
    registerStateCommand(State.NO_RING, setLights(State.NO_RING));
    registerStateCommand(State.HAVE_RING, setLights(State.HAVE_RING));
    registerStateCommand(State.TARGETING, setLights(State.TARGETING));
    registerStateCommand(State.READY, setLights(State.READY));
    registerStateCommand(State.INTAKE, setLights(State.INTAKE));
    registerStateCommand(State.AUTOMATIC_SCORE, setLights(State.AUTOMATIC_SCORE));
    registerStateCommand(State.EJECT, setLights(State.EJECT));
    registerStateCommand(State.CLIMB, setLights(State.CLIMB));
    registerStateCommand(State.ERROR, setLights(State.ERROR));
  }

  @Override
  protected void update() {
    io.updateInputs(inputs);
    super.update();
  }

  private Command setLights(State state) {
    return new WhileDisabledInstantCommand(() -> state.data.applyToCANdle(io));
  }

  @Override
  protected void determineSelf() {
    setState(State.RESTING);
  }

  public enum State {
    UNDETERMINED(new LEDData(DISABLED_ANIMATION)),
    RESTING(new LEDData(DISABLED_ANIMATION)), // Disabled
    AUTO(new LEDData(AUTO_ANIMATION)), // Autonomous
    NO_RING(new LEDData(NO_RING_RGB)),
    HAVE_RING(new LEDData(HOLDING_RING)),
    TARGETING(new LEDData(TARGETING_ANIMATION)),
    READY(new LEDData(READY_TO_SHOOT)),
    INTAKE(new LEDData(INTAKE_ANIMATION)),
    AUTOMATIC_SCORE(new LEDData(AUTOMATIC_SCORE_ANIMATION)),
    EJECT(new LEDData(EJECT_ANIMATION)),
    CLIMB(new LEDData(CLIMB_RGB)),
    ERROR(new LEDData(ERROR_RGB));

    private final LEDData data;

    State(LEDData data) {
      this.data = data;
    }
  }
}
