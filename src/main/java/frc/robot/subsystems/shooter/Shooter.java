package frc.robot.subsystems.shooter;

import static frc.robot.Constants.Shooter.Settings.*;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.ShamLib.SMF.StateMachine;
import frc.robot.subsystems.shooter.arm.Arm;
import frc.robot.subsystems.shooter.arm.ArmIO;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.flywheel.FlywheelIO;
import java.util.Map;
import java.util.NavigableMap;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class Shooter extends StateMachine<Shooter.State> {
  private final Arm arm;
  private final Flywheel flywheel;

  // odom
  private final Supplier<Translation3d> botTranslationProvider;

  // angle about x axis from gyro
  private final DoubleSupplier botXAngleProvider;

  // climber extension
  private final DoubleSupplier climberExtensionSupplier;

  public Shooter(
      ArmIO armIO,
      FlywheelIO flywheelIO,
      Supplier<Translation3d> botTranslationProvider,
      DoubleSupplier botXAngleProvider,
      DoubleSupplier climberExtensionSupplier,
      Trigger tuningInc,
      Trigger tuningDec,
      Trigger tuningStop) {
    super("Shooter", State.UNDETERMINED, State.class);

    this.botTranslationProvider = botTranslationProvider;
    this.botXAngleProvider = botXAngleProvider;
    this.climberExtensionSupplier = climberExtensionSupplier;

    arm =
        new Arm(
            armIO,
            () -> distanceAA(ARM_DISTANCE_LUT, Constants.Arm.Settings.BASE_SHOT_POSITION),
            this::armTrapAA,
            tuningInc,
            tuningDec,
            tuningStop);

    flywheel =
        new Flywheel(
            flywheelIO,
            () -> distanceAA(FLYWHEEL_DISTANCE_LUT, Constants.Flywheel.Settings.BASE_SHOT_VELOCITY),
            tuningInc,
            tuningDec,
            tuningStop);

    addChildSubsystem(arm);
    addChildSubsystem(flywheel);

    registerStateCommands();
    registerTransitions();
  }

  private void registerStateCommands() {
    registerStateCommand(
        State.SOFT_E_STOP,
        new ParallelCommandGroup(
            arm.transitionCommand(Arm.State.SOFT_E_STOP),
            flywheel.transitionCommand(Flywheel.State.IDLE)));

    registerStateCommand(
        State.BASE_SHOT,
        new ParallelCommandGroup(
            arm.transitionCommand(Arm.State.BASE_SHOT),
            flywheel.transitionCommand(Flywheel.State.BASE_SHOT_SPIN),
            watchReadyCommand()));

    registerStateCommand(
        State.CHUTE_INTAKE,
        new ParallelCommandGroup(
            arm.transitionCommand(Arm.State.CHUTE_INTAKE),
            flywheel.transitionCommand(Flywheel.State.CHUTE_INTAKE),
            watchReadyCommand()));

    registerStateCommand(
        State.STOW,
        new ParallelCommandGroup(
            arm.transitionCommand(Arm.State.FULL_STOW),
            flywheel.transitionCommand(Flywheel.State.IDLE),
            watchReadyCommand()));

    registerStateCommand(
        State.TRAP,
        new ParallelCommandGroup(
            arm.transitionCommand(Arm.State.TRAP_ACTIVE_ADJUST),
            flywheel.transitionCommand(Flywheel.State.PASS_THROUGH),
            watchReadyCommand()));

    registerStateCommand(
        State.TRAP_PREP,
        new ParallelCommandGroup(
            arm.transitionCommand(Arm.State.TRAP_PREP),
            flywheel.transitionCommand(Flywheel.State.PASS_THROUGH),
            watchReadyCommand()));

    registerStateCommand(
        State.PARTIAL_STOW,
        new ParallelCommandGroup(
            arm.transitionCommand(Arm.State.PARTIAL_STOW),
            flywheel.transitionCommand(Flywheel.State.IDLE),
            watchReadyCommand()));

    registerStateCommand(
        State.ARM_VOLTAGE_CALC,
        new ParallelCommandGroup(
                flywheel.transitionCommand(Flywheel.State.IDLE),
                arm.transitionCommand(Arm.State.VOLTAGE_CALC))
            .andThen(arm.waitForState(Arm.State.SOFT_E_STOP))
            .andThen(transitionCommand(State.SOFT_E_STOP)));

    registerStateCommand(
        State.FLYWHEEL_VOLTAGE_CALC,
        new ParallelCommandGroup(
                arm.transitionCommand(Arm.State.SOFT_E_STOP),
                flywheel.transitionCommand(Flywheel.State.VOLTAGE_CALC))
            .andThen(flywheel.waitForState(Flywheel.State.IDLE))
            .andThen(transitionCommand(State.SOFT_E_STOP)));
  }

  private void registerTransitions() {
    // omnis cause none of these states conflict with anything within the subsystem
    addOmniTransition(State.SOFT_E_STOP);

    addOmniTransition(State.BASE_SHOT);
    addOmniTransition(State.STOW);
    addOmniTransition(State.PARTIAL_STOW);
    addOmniTransition(State.TRAP);
    addOmniTransition(State.TRAP_PREP);
    addOmniTransition(State.CHUTE_INTAKE);

    addTransition(State.SOFT_E_STOP, State.BOTTOM_FLYWHEEL_VOLTAGE_CALC);
    addTransition(State.SOFT_E_STOP, State.FLYWHEEL_VOLTAGE_CALC);
    addTransition(State.SOFT_E_STOP, State.ARM_VOLTAGE_CALC);
  }

  private Command watchReadyCommand() {
    return new RunCommand(
        () -> {
          if (arm.isFlag(Arm.State.AT_TARGET) && flywheel.isFlag(Flywheel.State.AT_SPEED)) {
            setFlag(State.READY);
          } else {
            clearFlag(State.READY);
          }
        });
  }

  private double armTrapAA() {
    double[] botTrapOffset =
        Constants.getTrapOffsetFromBot(
            climberExtensionSupplier.getAsDouble(), botXAngleProvider.getAsDouble());
    return Math.atan2(botTrapOffset[0], botTrapOffset[1]);
  }

  private double distanceAA(NavigableMap<Double, Double> map, double replacement) {
    double distance =
        Constants.PhysicalConstants.SPEAKER_POSE
            .getTranslation()
            .getDistance(botTranslationProvider.get());

    Map.Entry<Double, Double> lower = map.floorEntry(distance);
    Map.Entry<Double, Double> higher = map.ceilingEntry(distance);

    if (lower == null && higher == null) {
      // avoid crashing
      return replacement;
    } else if (lower == null || higher == null) {
      return lower != null ? lower.getValue() : higher.getValue();
    }

    double interpolation = (distance - lower.getKey()) / (higher.getKey() - lower.getKey());

    // TODO: check this math
    return Constants.lerp(lower.getValue(), higher.getValue(), interpolation);
  }

  @Override
  protected void determineSelf() {
    // await instructions from rc
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
    FLYWHEEL_VOLTAGE_CALC,
    BOTTOM_FLYWHEEL_VOLTAGE_CALC,
    ARM_VOLTAGE_CALC,
    // flags
    READY
  }
}
