package frc.robot.subsystems.climbers;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class AutomaticZeroCommand extends Command {

  private final Climber climber;
  private final ClimberIO climberIO;
  private final double runPower;
  private final double minRetractTime;
  private final double stallVeloThrehold;

  private final Timer timer = new Timer();

  public AutomaticZeroCommand(
      Climber climber,
      ClimberIO climberIO,
      double runPower,
      double minRetractTime,
      double stallVeloThreshold) {
    this.climberIO = climberIO;
    this.climber = climber;
    this.runPower = runPower;
    this.minRetractTime = minRetractTime;
    this.stallVeloThrehold = stallVeloThreshold;
  }

  @Override
  public void initialize() {
    climberIO.setPower(runPower);

    timer.restart();
  }

  @Override
  public boolean isFinished() {
    return timer.hasElapsed(minRetractTime) && climber.getClimberVelo() < stallVeloThrehold;
  }

  @Override
  public void end(boolean interrupted) {
    climberIO.stop();
    timer.stop();

    if (!interrupted) {
      climber.zero();
    }
  }
}
