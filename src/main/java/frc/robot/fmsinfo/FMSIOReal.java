package frc.robot.fmsinfo;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class FMSIOReal implements FMSIO {
  private final NetworkTable table;

  public FMSIOReal() {
    table = NetworkTableInstance.getDefault().getTable("FMSInfo");
  }

  @Override
  public void updateInputs(FMSInputs inputs) {

    inputs.connected = table.getKeys().size() > 0;

    if (inputs.connected) {
      inputs.isRedAlliance = table.getEntry("IsRedAlliance").getBoolean(true);

      inputs.eventName = table.getEntry("EventName").getString("");

      inputs.eventName = table.getEntry("EventName").getString("");
    }
  }
}
