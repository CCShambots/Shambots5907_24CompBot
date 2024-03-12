package frc.robot.subsystems.climbers;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class AutomaticZeroCommand extends Command {

  private final Climber climber;
  private final ClimberIO climberIO;
  private final double runPower;

  private final Timer timer = new Timer();

  public AutomaticZeroCommand(
      Climber climber,
      ClimberIO climberIO,
      double runPower) {
    this.climberIO = climberIO;
    this.climber = climber;
    this.runPower = runPower;
  }

  @Override
  public void initialize() {
    climberIO.setPower(runPower);

    timer.restart();
  }

  @Override
  public boolean isFinished() {
    System.out.println(climber.getClimberVelo());
    return climber.isTouchTripped();
  }

  @Override
  public void end(boolean interrupted) {
    climberIO.stop();

    if (!interrupted) {
      climber.zero();
    }
  }
}
