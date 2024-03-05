package frc.robot.subsystems.lights;

import static frc.robot.Constants.Lights.Settings.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.ShamLib.Candle.TimedColorFlowCommand;
import frc.robot.ShamLib.SMF.StateMachine;
import frc.robot.ShamLib.WhileDisabledInstantCommand;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class Lights extends StateMachine<Lights.State> {
  private final LightsIO io;
  private final LightsInputsAutoLogged inputs = new LightsInputsAutoLogged();

  private final BooleanSupplier flashAutoError;
  private final BooleanSupplier displayAutoInfo;

  public Lights(LightsIO io, BooleanSupplier flashAutoError, BooleanSupplier displayAutoInfo) {
    super("Lights", State.UNDETERMINED, State.class);

    this.io = io;

    this.flashAutoError = flashAutoError;
    this.displayAutoInfo = displayAutoInfo;

    registerStateCommmands();
    registerTransitions();
  }

  private void registerTransitions() {
    addOmniTransitions(State.values());
  }

  private void registerStateCommmands() {
    registerStandardState(State.NO_RING);
    registerStandardState(State.HAVE_RING);
    registerStandardState(State.TARGETING);
    registerStandardState(State.READY);
    registerStandardState(State.INTAKE);
    registerStandardState(State.AUTOMATIC_SCORE);
    registerStandardState(State.EJECT);
    registerStandardState(State.CLIMB);

    registerStateCommand(
        State.RESTING,
        new ParallelCommandGroup(
            setLights(State.RESTING),
            new SequentialCommandGroup(
                new WaitUntilCommand(displayAutoInfo),
                new WhileDisabledInstantCommand(() -> requestTransition(State.PRE_AUTO_REST)))));

    registerStateCommand(State.ERROR, new ParallelCommandGroup(setLights(State.ERROR)));

    registerStateCommand(
        State.PRE_AUTO_REST,
        new ParallelCommandGroup(
            setLights(State.PRE_AUTO_REST),
            new SequentialCommandGroup(
                new WaitUntilCommand(flashAutoError),
                new WhileDisabledInstantCommand(() -> requestTransition(State.AUTO_ERROR)))));

    registerStateCommand(
        State.AUTO_ERROR,
        new ParallelCommandGroup(
            setLights(State.AUTO_ERROR),
            new SequentialCommandGroup(
                new WaitUntilCommand(() -> !flashAutoError.getAsBoolean()),
                new WhileDisabledInstantCommand(() -> requestTransition(State.PRE_AUTO_REST)))));

    registerStateCommand(
        State.AUTO,
        new TimedColorFlowCommand(
            NUM_LIGHTS_WITHOUT_CANDLE,
            8,
            (segs) -> io.setMultipleSegs(segs),
            Constants.AUTO_TIME,
            AUTO_RGB,
            AUTO_BACKGROUND_RGB));
  }

  private void registerStandardState(State state) {
    registerStateCommand(state, setLights(state));
  }

  @Override
  protected void update() {
    io.updateInputs(inputs);
    super.update();

    Logger.recordOutput("display auto info", displayAutoInfo.getAsBoolean());
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
    PRE_AUTO_REST(new LEDData(DISABLED_ANIMATION)), // Disabled
    AUTO(new LEDData(AUTO_ANIMATION)), // Autonomous
    NO_RING(new LEDData(NO_RING_RGB)),
    HAVE_RING(new LEDData(HOLDING_RING)),
    TARGETING(new LEDData(TARGETING_ANIMATION)),
    READY(new LEDData(READY_TO_SHOOT)),
    INTAKE(new LEDData(INTAKE_ANIMATION)),
    AUTOMATIC_SCORE(new LEDData(AUTOMATIC_SCORE_ANIMATION)),
    EJECT(new LEDData(EJECT_ANIMATION)),
    CLIMB(new LEDData(CLIMB_RGB)),
    ERROR(new LEDData(ERROR_RGB)),
    AUTO_ERROR(new LEDData(ERROR_RGB));

    private final LEDData data;

    State(LEDData data) {
      this.data = data;
    }
  }
}
