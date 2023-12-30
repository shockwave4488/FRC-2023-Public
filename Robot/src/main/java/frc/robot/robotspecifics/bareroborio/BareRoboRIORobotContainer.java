package frc.robot.robotspecifics.bareroborio;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.BaseRobotContainer;
import frc.lib.dashboard.DashboardServer;
import frc.lib.dashboard.IDashboardSupportedContainer;
import frc.lib.dashboard.gui.ButtonWidget;
import frc.lib.dashboard.gui.CameraWidget;
import frc.lib.dashboard.gui.DisplayWidget;
import frc.lib.dashboard.gui.DropdownWidget;
import frc.lib.dashboard.gui.FieldWidget;
import frc.lib.dashboard.gui.GroupWidget;
import frc.lib.dashboard.gui.SwitchWidget;
import frc.lib.logging.LogManager;
import frc.lib.preferences.PreferencesParser;
import java.util.List;

public class BareRoboRIORobotContainer extends BaseRobotContainer
    implements IDashboardSupportedContainer {

  public BareRoboRIORobotContainer(PreferencesParser prefs, LogManager logger) {
    super(prefs, logger);
  }

  public Command getAutonomousCommand() {
    return null;
  }

  protected void addSubsystems() {}

  @Override
  public void onDashboardInit(DashboardServer dashboard) {
    // Dummy field for testing dashboard widgets
    Field2d field = new Field2d();

    GroupWidget root = dashboard.getWebsite().getWidgets();
    root.setDirection(GroupWidget.GroupWidgetDirection.VERTICAL);

    GroupWidget header = new GroupWidget(GroupWidget.GroupWidgetDirection.HORIZONTAL, false, true);
    GroupWidget body = new GroupWidget(GroupWidget.GroupWidgetDirection.HORIZONTAL, true, true);
    header.setSizeLocked(true);
    root.addWidget(header);
    root.addWidget(body);

    header.addWidget(new DisplayWidget("Commands: {/LiveWindow/Ungrouped/CommandLogger/Names}"));

    GroupWidget leftGroup = new GroupWidget(GroupWidget.GroupWidgetDirection.VERTICAL, true, true);
    GroupWidget rightGroup =
        new GroupWidget(GroupWidget.GroupWidgetDirection.VERTICAL, false, true);
    body.addWidget(leftGroup);
    body.addWidget(rightGroup);

    leftGroup.addWidget(new CameraWidget("{11}:5800", true));
    leftGroup.addWidget(new CameraWidget("{2}:1181", true));

    rightGroup.addWidget(new FieldWidget("Field", FieldWidget.FieldYear.Y2023));
    field.getObject("Test2").setPose(5, 2.5, Rotation2d.fromDegrees(-135));
    field
        .getObject("[Trajectory] Traj")
        .setTrajectory(
            TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, Rotation2d.fromDegrees(45)),
                List.of(),
                new Pose2d(2.5, 5, Rotation2d.fromDegrees(90)),
                new TrajectoryConfig(5, 5)));

    GroupWidget options = new GroupWidget(GroupWidget.GroupWidgetDirection.VERTICAL, false, false);
    rightGroup.addWidget(options);

    DropdownWidget<String> dropdown = new DropdownWidget<>("SendableChooser[1]");
    dropdown.addOption("test", "test_value");
    dropdown.addOption("test2", "test2_value");
    options.addWidget(dropdown);

    options.addWidget(
        new ButtonWidget(
            "println(hi)",
            dashboard,
            () -> {
              dashboard.println("hi");
              System.out.println("hi");
            }));

    SwitchWidget switch1 = new SwitchWidget("Switch1", false);
    SwitchWidget switch2 = new SwitchWidget("Switch2", true);
    options.addWidget(switch1);
    options.addWidget(switch2);

    SmartDashboard.putData("Field", field);
    new Thread(
            () -> {
              while (true) {
                try {
                  Thread.sleep(20);
                } catch (InterruptedException e) {
                  return;
                }
                field
                    .getObject("Test")
                    .setPoses(
                        new Pose2d(5, 5, Rotation2d.fromRotations(0.25)),
                        new Pose2d(10, 5, Rotation2d.fromRotations(Timer.getFPGATimestamp() / 2)));
                System.out.println(
                    dropdown.getSelected()
                        + " - "
                        + switch1.isEnabled()
                        + " - "
                        + switch2.isEnabled());
              }
            })
        .start();
  }
}
