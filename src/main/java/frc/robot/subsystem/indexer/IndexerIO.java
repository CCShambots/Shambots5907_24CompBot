package frc.robot.subsystem.indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
  @AutoLog
  public static class IndexerIOInputs {
    public double beltVelocity = 0.0; // DEG/s
    public double beltTargetVelocity = 0.0; // DEG/s

    public boolean[] proximitySensors = {false, false, false};
  }

  public void stop();

  public void setTargetVelocity(double targetVelocity);

  public void updateInputs(IndexerIOInputs inputs);
}
