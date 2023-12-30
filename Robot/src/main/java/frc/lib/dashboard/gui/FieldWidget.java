package frc.lib.dashboard.gui;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.Constants2022;
import frc.robot.constants.Constants2023;
import java.io.IOException;
import org.msgpack.core.MessagePacker;

public class FieldWidget extends Widget {

  public enum FieldYear {
    Y2023(
        2023, Constants2023.FieldConstants.FIELD_LENGTH, Constants2023.FieldConstants.FIELD_WIDTH),
    /** Positions on the field seem to slightly disagree with Shuffleboard */
    Y2022(
        2022,
        Constants2022.FieldConstants.HUB_CENTER.getX() * 2,
        Constants2022.FieldConstants.HUB_CENTER.getY() * 2);

    private final short year;
    private final double width;
    private final double height;

    private FieldYear(int year, double width, double height) {
      this.year = (short) year;
      this.width = width;
      this.height = height;
    }
  }

  private final String name;
  private final FieldYear year;

  /**
   * Use {@link Field2d} and send it separately with the same name To render a trajectory, use
   * "[Trajectory] ObjectName" instead of "ObjectName" Note: This widget uses the faster
   * NetworkTables update period specified in index.html
   *
   * @param name The name of the {@link SmartDashboard} entry
   */
  public FieldWidget(String name, FieldYear year) {
    super("field");
    this.name = name;
    this.year = year;
  }

  @Override
  protected void sendData(MessagePacker packer) throws IOException {
    packer.packString(name);
    packer.packShort(year.year);
    packer.packDouble(year.width);
    packer.packDouble(year.height);
  }
}
