package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants;
import frc.robot.ShamLib.SMF.StateMachine;
import frc.robot.subsystems.shooter.arm.Arm;
import frc.robot.subsystems.shooter.arm.ArmIO;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.flywheel.FlywheelIO;

import java.util.function.Supplier;

public class Shooter extends StateMachine<Shooter.State> {
  private final Arm arm;
  private final Flywheel flywheel;

  private final Supplier<Pose3d> botPoseProvider;

  public Shooter(ArmIO armIO, FlywheelIO flywheelIO, Supplier<Pose3d> botPoseProvider) {
    super("Shooter", State.UNDETERMINED, State.class);

    this.botPoseProvider = botPoseProvider;

    arm = new Arm(
            armIO,
            this::armDistanceAA,
            this::armTrapAA
    );

    flywheel = new Flywheel(
            flywheelIO,
            this::flywheelDistanceAA
    );

    addChildSubsystem(arm);
    addChildSubsystem(flywheel);

    registerStateCommands();
    registerTransitions();
  }

  private void registerStateCommands() {
    registerStateCommand(State.SOFT_E_STOP, new ParallelCommandGroup(
            arm.transitionCommand(Arm.State.SOFT_E_STOP),
            flywheel.transitionCommand(Flywheel.State.IDLE)
    ));

    registerStateCommand(State.BASE_SHOT, new ParallelCommandGroup(
            arm.transitionCommand(Arm.State.BASE_SHOT),
            flywheel.transitionCommand(Flywheel.State.BASE_SHOT_SPIN),
            watchReadyCommand()
    ));

    registerStateCommand(State.CHUTE_INTAKE, new ParallelCommandGroup(
            arm.transitionCommand(Arm.State.CHUTE_INTAKE),
            flywheel.transitionCommand(Flywheel.State.CHUTE_INTAKE),
            watchReadyCommand()
    ));

    registerStateCommand(State.STOW, new ParallelCommandGroup(
            arm.transitionCommand(Arm.State.FULL_STOW),
            flywheel.transitionCommand(Flywheel.State.IDLE),
            watchReadyCommand()
    ));

    registerStateCommand(State.TRAP, new ParallelCommandGroup(
            arm.transitionCommand(Arm.State.TRAP_ACTIVE_ADJUST),
            flywheel.transitionCommand(Flywheel.State.PASS_THROUGH),
            watchReadyCommand()
    ));

    registerStateCommand(State.TRAP_PREP, new ParallelCommandGroup(
            arm.transitionCommand(Arm.State.TRAP_PREP),
            flywheel.transitionCommand(Flywheel.State.PASS_THROUGH),
            watchReadyCommand()
    ));

    registerStateCommand(State.PARTIAL_STOW, new ParallelCommandGroup(
            arm.transitionCommand(Arm.State.PARTIAL_STOW),
            flywheel.transitionCommand(Flywheel.State.IDLE),
            watchReadyCommand()
    ));
  }

  private void registerTransitions() {
    //omnis cause none of these states conflict with anything within the subsystem
    addOmniTransition(State.SOFT_E_STOP);

    addOmniTransition(State.BASE_SHOT);
    addOmniTransition(State.STOW);
    addOmniTransition(State.PARTIAL_STOW);
    addOmniTransition(State.TRAP);
    addOmniTransition(State.TRAP_PREP);
    addOmniTransition(State.CHUTE_INTAKE);
  }

  private Command watchReadyCommand() {
    return new RunCommand(() -> {
      if (arm.isFlag(Arm.State.AT_TARGET) && flywheel.isFlag(Flywheel.State.AT_SPEED)) {
        setFlag(State.READY);
      }
      else {
        clearFlag(State.READY);
      }
    });
  }

  private double armTrapAA() {
    //TODO: do math probably
    return Constants.Arm.Settings.TRAP_PREP_POSITION;
  }

  private double armDistanceAA() {
    //TODO: do math probably
    return Constants.Arm.Settings.BASE_SHOT_POSITION;
  }

  private double flywheelDistanceAA() {
    //TODO: maybe do math???
    return Constants.Flywheel.Settings.BASE_SHOT_VELOCITY;
  }

  @Override
  protected void determineSelf() {
    //await instructions from rc
    setState(State.SOFT_E_STOP);
  }

  public enum State {
    UNDETERMINED,
    SOFT_E_STOP,
    BASE_SHOT,
    STOW,
    PARTIAL_STOW,
    CHUTE_INTAKE,
    TRAP,
    TRAP_PREP,
    //flags
    READY
  }
}
