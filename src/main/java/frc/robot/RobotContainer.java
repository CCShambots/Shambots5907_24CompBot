// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.ShamLib.SMF.StateMachine;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOReal;
import frc.robot.subsystems.intake.IntakeIOSim;

public class RobotContainer extends StateMachine<RobotContainer.State> {
  private final CommandXboxController operatorController = new CommandXboxController(0);
  private final Intake intake;

  public RobotContainer() {
    super("Robot Container", State.UNDETERMINED, State.class);

    intake = new Intake(getIntakeIO(), operatorController.povUp());

    addChildSubsystem(intake);

    SmartDashboard.putData("intake", intake);

    configureBindings();
  }

  private void configureBindings() {
    operatorController.a().onTrue(intake.transitionCommand(Intake.State.SEEKING_STOWED_NO_EXPEL));
    operatorController.b().onTrue(intake.transitionCommand(Intake.State.SEEKING_DEPLOYED));
    operatorController.y().onTrue(intake.transitionCommand(Intake.State.SOFT_E_STOP));
    operatorController.x().onTrue(intake.transitionCommand(Intake.State.SEEKING_STOWED_EXPEL));
    operatorController.povUp().onTrue(intake.transitionCommand(Intake.State.MANUAL_CONTROL));
  }

  private IntakeIO getIntakeIO() {
    switch (Constants.currentBuildMode) {
      case REAL -> {
        return new IntakeIOReal();
      }
      case SIM -> {
        return new IntakeIOSim();
      }
      default -> {
        return new IntakeIO() {};
      }
    }
  }

  public Pose3d[] getComponentPositions() {
    return new Pose3d[] {intake.getArmPose(), intake.getBeltPose()};
  }

  public Pose3d[] getComponentTargetPositions() {
    return new Pose3d[] {intake.getArmTargetPose(), intake.getBeltTargetPose()};
  }

  @Override
  protected void onEnable() {
    intake.requestTransition(Intake.State.SEEKING_STOWED_NO_EXPEL);
  }

  @Override
  protected void determineSelf() {
    // placeholder
    setState(State.IDLE);
  }

  public enum State {
    UNDETERMINED,
    IDLE // placeholder
  }
}
