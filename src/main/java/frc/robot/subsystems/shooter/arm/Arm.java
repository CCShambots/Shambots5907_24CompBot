package frc.robot.subsystems.shooter.arm;

import static frc.robot.Constants.Arm.Settings.*;
import static frc.robot.Constants.doubleEqual;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.ShamLib.SMF.StateMachine;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Arm extends StateMachine<Arm.State> {
  private final ArmIO io;
  private final ArmInputsAutoLogged inputs = new ArmInputsAutoLogged();

  // AA stands for active adjust :)
  private final DoubleSupplier distanceAAProvider;
  private final DoubleSupplier trapAAProvider;

  public Arm(ArmIO io, DoubleSupplier distanceAAProvider, DoubleSupplier trapAAProvider) {
    super("Shooter Arm", State.UNDETERMINED, State.class);

    this.io = io;
    this.distanceAAProvider = distanceAAProvider;
    this.trapAAProvider = trapAAProvider;
  }

  private void registerStateCommands() {
    registerStateCommand(State.SOFT_E_STOP, io::stop);

    registerStateCommand(State.AMP, holdPositionCommand(() -> AMP_POSITION));
    registerStateCommand(State.BASE_SHOT, holdPositionCommand(() -> BASE_SHOT_POSITION));
    registerStateCommand(State.CHUTE_INTAKE, holdPositionCommand(() -> CHUTE_INTAKE_POSITION));
    registerStateCommand(State.PARTIAL_STOW, holdPositionCommand(() -> PARTIAL_STOW_POSITION));
    registerStateCommand(State.FULL_STOW, holdPositionCommand(() -> FULL_STOW_POSITION));
    registerStateCommand(State.TRAP_PREP, holdPositionCommand(() -> TRAP_PREP_POSITION));

    registerStateCommand(State.TRAP_ACTIVE_ADJUST, holdPositionCommand(trapAAProvider));
    registerStateCommand(State.SHOT_ACTIVE_ADJUST, holdPositionCommand(distanceAAProvider));
  }

  private void registerTransitions() {
    addOmniTransition(State.SOFT_E_STOP);

    // it going from one to the other wont conflict with anything within the arm subsystem
    addOmniTransition(State.AMP);
    addOmniTransition(State.BASE_SHOT);
    addOmniTransition(State.CHUTE_INTAKE);
    addOmniTransition(State.PARTIAL_STOW);
    addOmniTransition(State.FULL_STOW);
    addOmniTransition(State.TRAP_PREP);
    addOmniTransition(State.TRAP_ACTIVE_ADJUST);
    addOmniTransition(State.SHOT_ACTIVE_ADJUST);
  }

  private Command holdPositionCommand(DoubleSupplier positionProvider) {
    return new ParallelCommandGroup(
        new RunCommand(
            () -> {
              double target = positionProvider.getAsDouble();

              // avoid spamming can network :)
              if (!doubleEqual(target, inputs.targetPosition)) {
                io.setTargetPosition(target);
              }
            }),
        atTargetCommand(positionProvider));
  }

  private Command atTargetCommand(DoubleSupplier positionProvider) {
    return new RunCommand(
        () -> {
          if (doubleEqual(
              inputs.motorPosition, positionProvider.getAsDouble(), POSITION_READY_TOLERANCE)) {
            setFlag(State.AT_TARGET);
          } else {
            clearFlag(State.AT_TARGET);
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
    SOFT_E_STOP,
    AMP,
    CHUTE_INTAKE,
    TRAP_PREP,
    TRAP_ACTIVE_ADJUST,
    BASE_SHOT,
    SHOT_ACTIVE_ADJUST,
    PARTIAL_STOW,
    FULL_STOW,
    // flags
    AT_TARGET
  }
}
