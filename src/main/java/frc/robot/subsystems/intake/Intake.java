package frc.robot.subsystems.intake;

import static frc.robot.Constants.Intake.Settings.*;
import static frc.robot.Constants.doubleEqual;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.ShamLib.SMF.StateMachine;
import java.util.concurrent.atomic.AtomicBoolean;
import org.littletonrobotics.junction.Logger;

public class Intake extends StateMachine<Intake.State> {
  private final IntakeIO io;
  private final Timer syncTimeout = new Timer();

  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  public Intake(IntakeIO io, Trigger manualControlBeltToggle) {
    super("Intake", State.UNDETERMINED, State.class);

    this.io = io;

    io.updateInputs(inputs);

    registerStateCommands(manualControlBeltToggle);
    registerTransitions();
  }

  private void registerStateCommands(Trigger beltToggle) {
    registerStateCommand(
        State.SEEKING_STOWED_NO_EXPEL,
        new SequentialCommandGroup(goToAngleCommand(STOW_ANGLE), transitionCommand(State.STOWED)));

    registerStateCommand(
        State.SEEKING_STOWED_EXPEL,
        new SequentialCommandGroup(
            new ParallelCommandGroup(
                goToAngleCommand(STOW_ANGLE),
                runBeltsCommand(-BELT_SPEED).withTimeout(STOW_EXPEL_DURATION)),
            transitionCommand(State.STOWED)));

    registerStateCommand(
        State.SEEKING_DEPLOYED,
        new SequentialCommandGroup(
            goToAngleCommand(DEPLOY_ANGLE), transitionCommand(State.DEPLOYED)));

    registerStateCommand(State.DEPLOYED, runBeltsCommand(BELT_SPEED));

    registerStateCommand(
        State.SOFT_E_STOP,
        new InstantCommand(
            () -> {
              io.stopArm();
              io.stopBelt();
            }));

    registerStateCommand(State.MANUAL_CONTROL, manualControlCommand(beltToggle));
  }

  private void registerTransitions() {
    addTransition(State.SEEKING_STOWED_EXPEL, State.STOWED);
    addTransition(State.SEEKING_STOWED_NO_EXPEL, State.STOWED);

    addOmniTransition(State.SEEKING_STOWED_NO_EXPEL);
    addOmniTransition(State.SEEKING_DEPLOYED);

    // avoid expelling whenever
    addTransition(State.DEPLOYED, State.SEEKING_STOWED_EXPEL);

    addTransition(State.SEEKING_DEPLOYED, State.DEPLOYED);

    addOmniTransition(State.SOFT_E_STOP);
    addOmniTransition(State.MANUAL_CONTROL);
  }

  private Command manualControlCommand(Trigger beltTrigger) {
    AtomicBoolean runBelt = new AtomicBoolean(true);

    // i know this isnt atomic, i cant for the life of me figure out a simple way to atomically
    // negate an atomic boolean in java
    beltTrigger.onTrue(new InstantCommand(() -> runBelt.set(!runBelt.get())));

    return new FunctionalCommand(
        () -> runBelt.set(false),
        () -> {
          if (runBelt.get() && !doubleEqual(inputs.beltTargetVelocity, BELT_SPEED)) {
            io.setBeltTargetVelocity(BELT_SPEED);
          } else if (!runBelt.get() && !doubleEqual(inputs.beltTargetVelocity, 0)) {
            io.setBeltTargetVelocity(0);
          }
        },
        (interrupted) -> io.setBeltTargetVelocity(0),
        () -> false);
  }

  private Command runBeltsCommand(double velocity) {
    return new FunctionalCommand(
        () -> io.setBeltTargetVelocity(velocity),
        () -> {},
        (interrupted) -> io.setBeltTargetVelocity(0),
        () -> false);
  }

  private Command goToAngleCommand(double state) {
    return new FunctionalCommand(
        () -> io.setArmTargetPosition(state),
        () -> {},
        (interrupted) -> {},
        () -> doubleEqual(inputs.armPosition, state, ANGLE_SETPOINT_TOLERANCE));
  }

  private boolean needsSync() {
    // arm motor position and abs encoder position are outside of tolerance as well as making sure
    // the arm isn't moving very fast
    return !doubleEqual(inputs.armPosition, inputs.absoluteEncoderPosition, AUTO_SYNC_TOLERANCE)
        && doubleEqual(inputs.armVelocity, 0.0, 1.0);
  }

  public Pose3d getArmPose() {
    return new Pose3d(
        new Translation3d(0, 0, 0), new Rotation3d(inputs.armPosition * (Math.PI / 180.0), 0, 0));
  }

  public Pose3d getArmTargetPose() {
    return new Pose3d(
        new Translation3d(0, 0, 0),
        new Rotation3d(inputs.armTargetPosition * (Math.PI / 180.0), 0, 0));
  }

  public Pose3d getBeltPose() {
    return new Pose3d(
        new Translation3d(
            0,
            0.3 * Math.cos(inputs.armPosition * (Math.PI / 180.0)),
            0.3 * Math.sin(inputs.armPosition * (Math.PI / 180.0))),
        new Rotation3d(inputs.beltPosition * (Math.PI * 2), 0, 0));
  }

  public Pose3d getBeltTargetPose() {
    return new Pose3d(
        new Translation3d(
            0,
            0.3 * Math.cos(inputs.armTargetPosition) * (Math.PI / 180.0),
            0.3 * Math.sin(inputs.armTargetPosition)),
        new Rotation3d(inputs.armPosition * (Math.PI / 180.0), 0, 0));
  }

  @Override
  protected void update() {
    io.updateInputs(inputs);
    Logger.processInputs(getName(), inputs);

    // sync the motor to the absolute encoder if needed (and feature is enabled)
    // there is a configurable timer so we don't spam the can network
    if (USE_AUTO_SYNC
        && needsSync()
        && syncTimeout.hasElapsed(MINIMUM_TIME_BETWEEN_SYNC_ATTEMPTS)) {
      io.syncToAbsoluteEncoder();
      syncTimeout.restart();
    }
  }

  @Override
  protected void determineSelf() {
    // wait for rc to orchestrate things
    setState(State.SOFT_E_STOP);
  }

  @Override
  protected void onEnable() {
    io.syncToAbsoluteEncoder();
  }

  public enum State {
    UNDETERMINED,
    STOWED,
    SEEKING_STOWED_EXPEL,
    SEEKING_STOWED_NO_EXPEL,
    DEPLOYED,
    SEEKING_DEPLOYED,
    SOFT_E_STOP,
    MANUAL_CONTROL
  }
}
