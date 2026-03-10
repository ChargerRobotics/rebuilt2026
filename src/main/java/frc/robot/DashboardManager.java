package frc.robot;

import java.util.Optional;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;
import org.littletonrobotics.junction.networktables.LoggedNetworkString;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.led.RobotState;

public class DashboardManager {
  private final RobotContainer robot;

  private final LoggedNetworkNumber matchTime = new LoggedNetworkNumber("Math Time");
  private final LoggedNetworkString match = new LoggedNetworkString("Match");
  private final LoggedNetworkString alliance = new LoggedNetworkString("Alliance");
  private final LoggedNetworkString state = new LoggedNetworkString("Robot State");
  private final LoggedNetworkNumber periodTime = new LoggedNetworkNumber("Period Time");
  private final LoggedNetworkString timeframe = new LoggedNetworkString("Timeframe");
  private final LoggedNetworkString activeHub = new LoggedNetworkString("Active Hub");
  private final Field2d field = new Field2d();

  public DashboardManager(RobotContainer robot) {
    this.robot = robot;

    SmartDashboard.putData("Swerve", robot.getDrive());
    SmartDashboard.putData("Field", field);
  }

  public void updateValues() {
    matchTime.set(DriverStation.getMatchTime());
    match.set(DriverStation.getMatchType().toString() + " " + DriverStation.getMatchNumber());

    Optional<Alliance> alliance = DriverStation.getAlliance();
    String color = "#888";
    if (alliance.isPresent()) {
      switch (alliance.get()) {
        case Red -> color = "#f0120f";
        case Blue -> color = "#4047ed";
      }
    }
    this.alliance.set(color);

    field.setRobotPose(robot.getDrive().getPose());

    RobotState state = robot.getLed().getCurrentState();
    this.state.set(state == null ? "NONE" : state.toString());

    double periodTime = -1;
    String timeframe = "NONE";
    if (DriverStation.isTeleop()) {
      double matchTime = DriverStation.getMatchTime();
      if (matchTime > secondsFromTimestamp(2, 10)) {
        // TRANSITION SHIFT (2:20-2:10)
        periodTime = matchTime - secondsFromTimestamp(2, 10);
        timeframe = "TRANSITION SHIFT";
      } else if (matchTime > secondsFromTimestamp(1, 45)) {
        // SHIFT 1 (2:10-1:45)
        periodTime = matchTime - secondsFromTimestamp(1, 45);
        timeframe = "SHIFT 1";
      } else if (matchTime > secondsFromTimestamp(1, 20)) {
        // SHIFT 2 (1:45-1:20)
        periodTime = matchTime - secondsFromTimestamp(1, 20);
        timeframe = "SHIFT 2";
      } else if (matchTime > secondsFromTimestamp(0, 55)) {
        // SHIFT 3 (1:20-0:55)
        periodTime = matchTime - secondsFromTimestamp(0, 55);
        timeframe = "SHIFT 3";
      } else if (matchTime > secondsFromTimestamp(0, 30)) {
        // SHIFT 4 (0:55-0:30)
        periodTime = matchTime - secondsFromTimestamp(0, 30);
        timeframe = "SHIFT 4";
      } else {
        // ENDGAME (0:30-0:00)
        periodTime = matchTime;
        timeframe = "ENDGAME";
      }
    }
    this.periodTime.set(periodTime);
    this.timeframe.set(timeframe);
  }

  private final double secondsFromTimestamp(int minutes, int seconds) {
    return minutes * 60 + seconds;
  }
}
