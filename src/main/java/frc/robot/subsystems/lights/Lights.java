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

  private final BooleanSupplier intakeTripped;
  private final BooleanSupplier leftClimbTripped;
  private final BooleanSupplier rightClimbTripped;

  public Lights(
      LightsIO io,
      BooleanSupplier flashAutoError,
      BooleanSupplier displayAutoInfo,
      BooleanSupplier intakeTripped,
      BooleanSupplier leftClimbTripped,
      BooleanSupplier rightClimbTripped) {
    super("Lights", State.UNDETERMINED, State.class);

    this.io = io;

    this.flashAutoError = flashAutoError;
    this.displayAutoInfo = displayAutoInfo;
    this.intakeTripped = intakeTripped;

    this.leftClimbTripped = leftClimbTripped;
    this.rightClimbTripped = rightClimbTripped;

    registerStateCommmands();
    registerTransitions();
  }

  private void registerTransitions() {
    addOmniTransitions(State.values());
  }

  private void registerStateCommmands() {

    registerStateCommand(
        State.NO_RING,
        new ParallelCommandGroup(
                setLights(getState()),
                new SequentialCommandGroup(
                    new WaitUntilCommand(intakeTripped),
                    setLights(State.PARTIAL_HOLD),
                    new WaitUntilCommand(() -> !intakeTripped.getAsBoolean())))
            .repeatedly());

    registerStateCommand(
        State.INTAKE,
        new ParallelCommandGroup(
                setLights(State.INTAKE),
                new SequentialCommandGroup(
                    new WaitUntilCommand(intakeTripped),
                    setLights(State.PARTIAL_INTAKE),
                    new WaitUntilCommand(() -> !intakeTripped.getAsBoolean())))
            .repeatedly());

    registerStandardState(State.HAVE_RING);
    registerStandardState(State.TARGETING);
    registerStandardState(State.READY);
    registerStandardState(State.AUTOMATIC_SCORE);
    registerStandardState(State.EJECT);
    registerStandardState(State.CLIMB);
    registerStandardState(State.GRAB_RANDOM_NOTE);

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

    registerStateCommand(
        State.TEST,
        new SequentialCommandGroup(
            setLights(State.TEST),
            new TestLightCommand(
                leftClimbTripped, rightClimbTripped, (info) -> info.applyToCANdle(io))));
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
    return new WhileDisabledInstantCommand(
        () -> {
          Logger.recordOutput("Lights/currentRGB", state.name());
          state.data.applyToCANdle(io);
        });
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
    GRAB_RANDOM_NOTE(new LEDData(GRAB_RANDOM_NOTE_ANIMATION)),
    TEST(new LEDData(OFF_RGB)),
    ERROR(new LEDData(ERROR_RGB)),
    AUTO_ERROR(new LEDData(ERROR_RGB)),
    PARTIAL_HOLD(new LEDData(PARTIAL_HOLD_RGB)),
    PARTIAL_INTAKE(new LEDData(PARTIAL_INTAKE_ANIAMTION));

    private final LEDData data;

    State(LEDData data) {
      this.data = data;
    }
  }
}
