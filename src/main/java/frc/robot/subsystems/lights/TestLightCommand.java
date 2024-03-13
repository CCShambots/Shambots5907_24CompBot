package frc.robot.subsystems.lights;

import static frc.robot.Constants.Lights.Settings.*;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

public class TestLightCommand extends Command {

  private final BooleanSupplier leftTouchPressedSupplier;
  private final BooleanSupplier rightTouchPressedSupplier;
  private final Consumer<LEDData> colorSegsAccepter;

  private final LEDData leftOnly = new LEDData(LEFT_CLIMB_TRIPPED);
  private final LEDData rightOnly = new LEDData(RIGHT_CLIMB_TRIPPED);
  private final LEDData both = new LEDData(CLIMB_TRIPPED_RGB);
  private final LEDData neither = new LEDData(OFF_RGB);

  public TestLightCommand(
      BooleanSupplier leftTouchPressedSupplier,
      BooleanSupplier rightTouchPressedSupplier,
      Consumer<LEDData> colorSegsAccepter) {
    this.leftTouchPressedSupplier = leftTouchPressedSupplier;
    this.rightTouchPressedSupplier = rightTouchPressedSupplier;
    this.colorSegsAccepter = colorSegsAccepter;
  }

  @Override
  public void execute() {
    if (leftTouchPressedSupplier.getAsBoolean() && rightTouchPressedSupplier.getAsBoolean()) {
      colorSegsAccepter.accept(both);
    } else if (leftTouchPressedSupplier.getAsBoolean()) {
      colorSegsAccepter.accept(leftOnly);
    } else if (rightTouchPressedSupplier.getAsBoolean()) {
      colorSegsAccepter.accept(rightOnly);
    } else {
      colorSegsAccepter.accept(neither);
    }
  }
}
