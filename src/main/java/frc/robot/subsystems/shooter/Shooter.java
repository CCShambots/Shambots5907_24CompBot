package frc.robot.subsystems.shooter;

import static frc.robot.Constants.Shooter.Settings.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DriverStation;
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
import frc.robot.util.StageSide;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class Shooter extends StateMachine<Shooter.State> {
  private final Arm arm;
  private final Flywheel flywheel;

  // odom
  private final Supplier<Translation2d> botTranslationProvider;

  // angle about x axis from gyro
  private final DoubleSupplier botXAngleProvider;

  // climber extension
  private final DoubleSupplier climberExtensionSupplier;

  private final Supplier<StageSide> targetStageSideSupplier;

  public Shooter(
      ArmIO armIO,
      FlywheelIO flywheelIO,
      Supplier<Translation2d> botTranslationProvider,
      DoubleSupplier botXAngleProvider,
      DoubleSupplier climberExtensionSupplier,
      Supplier<StageSide> targetStageSideSupplier,
      Trigger tuningInc,
      Trigger tuningDec,
      Trigger tuningStop) {
    super("Shooter", State.UNDETERMINED, State.class);

    this.botTranslationProvider = botTranslationProvider;
    this.botXAngleProvider = botXAngleProvider;
    this.climberExtensionSupplier = climberExtensionSupplier;
    this.targetStageSideSupplier = targetStageSideSupplier;

    arm =
        new Arm(
            armIO,
            () -> distanceAA(ARM_SPEAKER_DISTANCE_OFFSET_LUT),
            this::armTrapAA,
            tuningInc,
            tuningDec,
            tuningStop);

    flywheel =
        new Flywheel(
            flywheelIO,
            () -> distanceAA(FLYWHEEL_SPEAKER_DISTANCE_LUT),
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
        State.AMP,
        new ParallelCommandGroup(
            arm.transitionCommand(Arm.State.AMP),
            flywheel.transitionCommand(Flywheel.State.AMP),
            watchReadyCommand()));

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
    addOmniTransition(State.AMP);

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

  private double distanceAA(Pose2d pose, InterpolatingDoubleTreeMap map) {
    double distance =
        pose
            .getTranslation()
            .getDistance(botTranslationProvider.get());

    return map.get(distance);
  }

  private double armTrapAA() {
    double distance = getTrapDistance();

    //angle of shooter to face directly at trap target height plus lut offset
    return Math.atan2(TRAP_TARGET_HEIGHT, distance) + ARM_TRAP_DISTANCE_LUT.get(distance);
  }

  private double armSpeakerAA() {
    double distance = getSpeakerDistance();

    //angle of shooter to face directly at speaker target height plus lut offset
    return Math.atan2(SPEAKER_TARGET_HEIGHT, distance) + ARM_SPEAKER_DISTANCE_OFFSET_LUT.get(distance);
  }

  private double flywheelTrapAA() {
    return FLYWHEEL_TRAP_DISTANCE_LUT.get(getTrapDistance());
  }

  private double flywheelSpeakerAA() {
    return FLYWHEEL_SPEAKER_DISTANCE_LUT.get(getSpeakerDistance());
  }

  private double getSpeakerDistance() {
    Pose2d speaker = Constants.alliance == DriverStation.Alliance.Blue ?
            Constants.PhysicalConstants.BLUE_SPEAKER :
            Constants.mirror(Constants.PhysicalConstants.BLUE_SPEAKER);

    return speaker.getTranslation().getDistance(botTranslationProvider.get());
  }

  private double getTrapDistance() {
    Pose2d targetTrap = switch (targetStageSideSupplier.get()) {
      case CENTER -> Constants.PhysicalConstants.BLUE_CENTER_TRAP;
      case LEFT -> Constants.PhysicalConstants.BLUE_LEFT_TRAP;
      case RIGHT -> Constants.PhysicalConstants.BLUE_RIGHT_TRAP;
    };

    targetTrap = Constants.alliance == DriverStation.Alliance.Blue ? targetTrap : Constants.mirror(targetTrap);

    return targetTrap.getTranslation().getDistance(botTranslationProvider.get());
  }

  public double getArmAngle() {
    return arm.getAngle();
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
    AMP,
    // flags
    READY
  }
}
