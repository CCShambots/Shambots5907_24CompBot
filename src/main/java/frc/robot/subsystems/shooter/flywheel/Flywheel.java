package frc.robot.subsystems.shooter.flywheel;

import static frc.robot.Constants.Flywheel.Settings.*;
import static frc.robot.Constants.doubleEqual;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.ShamLib.SMF.StateMachine;
import frc.robot.ShamLib.motors.tuning.LinearTuningCommand;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Flywheel extends StateMachine<Flywheel.State> {
  private final FlywheelIO io;
  private final FlywheelInputsAutoLogged inputs = new FlywheelInputsAutoLogged();

  private final DoubleSupplier speakerAAProvider;

  public Flywheel(
      FlywheelIO io,
      DoubleSupplier speakerAAProvider,
      Trigger tuningInc,
      Trigger tuningDec,
      Trigger tuningStop) {
    super("Shooter Flywheel", State.UNDETERMINED, State.class);

    this.speakerAAProvider = speakerAAProvider;
    this.io = io;

    registerStateCommands(tuningInc, tuningDec, tuningStop);
    registerTransitions();

    SmartDashboard.putData("flywheel", this);
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
                () -> inputs.rotorVelocity,
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
        State.SPEAKER_ACTIVE_ADJUST_SPIN,
        new ParallelCommandGroup(
            new RunCommand(() -> io.setFlywheelTarget(speakerAAProvider.getAsDouble())),
            atSpeedCommand(speakerAAProvider, SPIN_UP_READY_TOLERANCE)));

    registerStateCommand(State.PASS_THROUGH, () -> io.setFlywheelTarget(PASS_THROUGH_SPEED));

    registerStateCommand(State.CHUTE_INTAKE, () -> io.setFlywheelTarget(CHUTE_INTAKE_SPEED));

    registerStateCommand(
        State.AMP,
        new ParallelCommandGroup(
            new InstantCommand(() -> io.setFlywheelTargets(AMP_SPEED_TOP, AMP_SPEED_BOTTOM)),
            atSpeedCommand(() -> AMP_SPEED_TOP, SPIN_UP_READY_TOLERANCE)));

    registerStateCommand(
        State.TRAP,
        new ParallelCommandGroup(
            new InstantCommand(() -> io.setFlywheelTargets(TRAP_SPEED_TOP, TRAP_SPEED_BOTTOM)),
            atSpeedCommand(() -> TRAP_SPEED_TOP, SPIN_UP_READY_TOLERANCE)));
  }

  private void registerTransitions() {
    // omni everything cause flywheel spinning around isnt gonna hurt anything
    addOmniTransition(State.IDLE);
    addOmniTransition(State.BASE_SHOT_SPIN);
    addOmniTransition(State.SPEAKER_ACTIVE_ADJUST_SPIN);
    addOmniTransition(State.TRAP);
    addOmniTransition(State.PASS_THROUGH);
    addOmniTransition(State.CHUTE_INTAKE);
    addOmniTransition(State.AMP);

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
    SPEAKER_ACTIVE_ADJUST_SPIN,
    TRAP,
    PASS_THROUGH,
    CHUTE_INTAKE,
    AMP,
    VOLTAGE_CALC,

    // flags
    AT_SPEED
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);

    builder.addDoubleProperty(
        "TRAP_TOP_SPEED",
        () -> Constants.Flywheel.Settings.TRAP_SPEED_TOP * 60.0,
        (val) -> {
          Constants.Flywheel.Settings.TRAP_SPEED_TOP = val / 60.0;
        });
    builder.addDoubleProperty(
        "TRAP_BOTTOM_SPEED",
        () -> Constants.Flywheel.Settings.TRAP_SPEED_BOTTOM * 60.0,
        (val) -> {
          Constants.Flywheel.Settings.TRAP_SPEED_BOTTOM = val / 60.0;
        });
  }
}
