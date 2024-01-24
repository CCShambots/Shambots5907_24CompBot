package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
  @AutoLog
  public static class IndexerInputs {
    public double beltVelocity = 0.0; // DEG/s
    public double beltTargetVelocity = 0.0; // DEG/s
    public double beltVoltage = 0.0;

    public boolean prox1 = false;
    public boolean prox2 = false;
  }

  public default void stop() {}
  ;

  public default void setTargetVelocity(double targetVelocity) {}
  ;

  public default void setVoltage(double voltage) {}
  ;

  public default void updateInputs(IndexerInputs inputs) {}
  ;
}
