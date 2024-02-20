// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.currentBuildMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.ShamLib.AllianceManager;
import frc.robot.ShamLib.SMF.SubsystemManagerFactory;
import frc.robot.ShamLib.ShamLibConstants;
import frc.robot.ShamLib.WhileDisabledInstantCommand;
import frc.robot.ShamLib.motors.talonfx.sim.PhysicsSim;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {
  private RobotContainer robotContainer;

  @AutoLogOutput private Pose3d armPose = new Pose3d();
  @AutoLogOutput private Pose3d botPose = new Pose3d();

  @AutoLogOutput private Pose3d[] componentPoses = new Pose3d[0];
  @AutoLogOutput private Pose2d botPose2D = new Pose2d();

  private int moduleCheckCounter = 0;

  private final EventLoop checkModulesLoop = new EventLoop();

  private boolean firstLoop = true;

  @Override
  public void robotInit() {
    if (isReal()) currentBuildMode = ShamLibConstants.BuildMode.REAL;

    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);

    switch (BuildConstants.DIRTY) {
      case 0:
        Logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        Logger.recordMetadata("GitDirty", "Uncomitted changes");
        break;
      default:
        Logger.recordMetadata("GitDirty", "Unknown");
        break;
    }

    switch (Constants.currentBuildMode) {
      case REAL:
        Logger.addDataReceiver(new WPILOGWriter("/home/lvuser/logs"));
        Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
        new PowerDistribution(1, PowerDistribution.ModuleType.kRev);
        break;
      case SIM:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new NT4Publisher());
        break;
      case REPLAY:
        setUseTiming(false); // Run as fast as possible
        String logPath =
            LogFileUtil
                .findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
        Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
        Logger.addDataReceiver(
            new WPILOGWriter(
                LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
        break;
    }

    Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may
    // be added.

    robotContainer = new RobotContainer(checkModulesLoop);

    SubsystemManagerFactory.getInstance().registerSubsystem(robotContainer, false);
    SubsystemManagerFactory.getInstance().disableAllSubsystems();

    // AKit shims the Driver Station using their logged driver station, so this shouldn't be a
    // problem
    new Trigger(
            () -> {
              if (firstLoop) {
                firstLoop = false;
                return false;
              } else {
                return true;
              }
            })
        .and(() -> DriverStation.isFMSAttached() || DriverStation.isDSAttached())
        .onTrue(
            new WaitCommand(2)
                .andThen(
                    new WhileDisabledInstantCommand(
                        () -> {
                          System.out.println("Getting alliance!");
                          AllianceManager.applyAlliance(DriverStation.getAlliance());
                          System.out.println("Alliance got: " + AllianceManager.getAlliance());
                        })));

    // Log Camera Poses
    Logger.recordOutput("Vision/limelight-pose", Constants.Vision.Hardware.RING_CAMERA_POSE);
    Logger.recordOutput("Vision/left-cam-pose", Constants.Vision.Hardware.LEFT_CAM_POSE);
    Logger.recordOutput("Vision/right-cam-pose", Constants.Vision.Hardware.RIGHT_CAM_POSE);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    updatePoses();

    moduleCheckCounter++;
    if (moduleCheckCounter >= 500) {
      moduleCheckCounter = 0;
      checkModulesLoop.poll();
    }
  }

  private void updatePoses() {
    armPose =
        new Pose3d()
            .transformBy(
                new Transform3d(new Pose3d(), Constants.PhysicalConstants.CHASSIS_TO_SHOOTER))
            .transformBy(
                new Transform3d(
                    new Pose3d(),
                    new Pose3d(0, 0, 0, new Rotation3d(0, 0, -robotContainer.getShooterAngle()))));

    botPose = robotContainer.getBotPose();

    botPose2D = botPose.toPose2d();
    componentPoses = new Pose3d[] {armPose};
  }

  @Override
  public void disabledInit() {
    SubsystemManagerFactory.getInstance().disableAllSubsystems();
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    SubsystemManagerFactory.getInstance().notifyAutonomousStart();
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    SubsystemManagerFactory.getInstance().notifyTeleopStart();
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {
    PhysicsSim.getInstance().run();
  }
}
