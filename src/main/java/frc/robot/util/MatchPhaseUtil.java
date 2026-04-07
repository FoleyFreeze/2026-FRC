package frc.robot.util;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class MatchPhaseUtil {
    public static enum MatchPhase {
        NONE,
        AUTO,
        PHASE_1,
        PHASE_2,
        PHASE_3,
        PHASE_4,
        ENDGAME
    }

public static MatchPhase matchPhaseState = MatchPhase.NONE;

//thank you wpilib for just having this <33
public boolean isHubActive() {
  Optional<Alliance> alliance = DriverStation.getAlliance();
  // If we have no alliance, we cannot be enabled, therefore no hub.
  if (alliance.isEmpty()) {
    return false;
  }
  // Hub is always enabled in autonomous.
  if (DriverStation.isAutonomousEnabled()) {
    return true;
  }
  // At this point, if we're not teleop enabled, there is no hub.
  if (!DriverStation.isTeleopEnabled()) {
    return false;
  }

  // We're teleop enabled, compute.
  double matchTime = DriverStation.getMatchTime();
  String gameData = DriverStation.getGameSpecificMessage();
  // If we have no game data, we cannot compute, assume hub is active, as its likely early in teleop.
  if (gameData.isEmpty()) {
    return true;
  }
  boolean redInactiveFirst = false;
  switch (gameData.charAt(0)) {
    case 'R' -> redInactiveFirst = true;
    case 'B' -> redInactiveFirst = false;
    default -> {
      // If we have invalid game data, assume hub is active.
      return true;
    }
  }

  // phase was is active for blue if red won auto, or red if blue won auto.
  boolean phase1Active = switch (alliance.get()) {
    case Red -> !redInactiveFirst;
    case Blue -> redInactiveFirst;
  };

  if (matchTime > 130) {
    // Transition phase, hub is active.
    return true;
  } else if (matchTime > 105) {
    // phase 1
    MatchPhaseUtil.matchPhaseState = MatchPhase.PHASE_1;
    return phase1Active;
  } else if (matchTime > 80) {
    // phase 2
    MatchPhaseUtil.matchPhaseState = MatchPhase.PHASE_2;
    return !phase1Active;
  } else if (matchTime > 55) {
    // phase 3
    MatchPhaseUtil.matchPhaseState = MatchPhase.PHASE_3;
    return phase1Active;
  } else if (matchTime > 30) {
    // phase 4
    MatchPhaseUtil.matchPhaseState = MatchPhase.PHASE_4;
    return !phase1Active;
  } else {
    // End game, hub always active.
    MatchPhaseUtil.matchPhaseState = MatchPhase.ENDGAME;
    return true;
        }
    }
}
