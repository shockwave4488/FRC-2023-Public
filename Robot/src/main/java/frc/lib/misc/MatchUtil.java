package frc.lib.misc;

import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.lib.commands.CommandLogger;
import frc.lib.logging.LogManager;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class MatchUtil {
  private MatchUtil() {}

  private static Optional<Boolean> inRealMatch =
      Optional.empty(); // empty Optional if search not finished yet
  private static final List<BooleanConsumer> uponRealMatchDeterminationFuncs = new ArrayList<>();
  private static boolean simulated;

  public enum MatchPhase {
    Autonomous,
    Teleop,
    Test;

    public static MatchPhase getCurrentPhase() {
      if (!DriverStation.isAutonomous()) {
        if (DriverStation.isTest()) {
          return MatchPhase.Test;
        } else {
          return MatchPhase.Teleop;
        }
      } else {
        return MatchPhase.Autonomous;
      }
    }

    public static Optional<MatchPhase> getCurrentPhaseEnabled() {
      return Optional.of(getCurrentPhase()).filter(phase -> DriverStation.isEnabled());
    }
  }

  public static Optional<Boolean> isRealMatch() {
    return inRealMatch;
  }

  public static void findIfRealMatch(LogManager logger, boolean simulated) {
    Command determineIfRealMatch =
        new WaitUntilCommand(DriverStation::isFMSAttached)
            .finallyDo(
                interrupted -> {
                  inRealMatch = Optional.of(!interrupted);
                  uponRealMatchDeterminationFuncs.forEach(func -> func.accept(!interrupted));
                })
            .until(DriverStation::isEnabled)
            .ignoringDisable(true);
    CommandLogger.getInstance().excludeCommand(determineIfRealMatch);
    determineIfRealMatch.schedule();
    MatchUtil.simulated = simulated;
  }

  public static void runOnRealMatchDetermination(BooleanConsumer func) {
    inRealMatch.ifPresentOrElse(func::accept, () -> uponRealMatchDeterminationFuncs.add(func));
  }

  public static void runOnRealMatchDetermination(Runnable func) {
    runOnRealMatchDetermination(realMatch -> func.run());
  }

  public static boolean isSimulated() {
    return simulated;
  }
}
