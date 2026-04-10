package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.junction.AutoLogOutput;

public class MatchPhaseUtil {

    @AutoLogOutput(key = "Timing/ReportedTime")
    private double lastMatchTime;

    @AutoLogOutput(key = "Timing/EstMatchTime")
    private double remainingTime;

    private double lastRobotTime;

    public static enum MatchPhase {
        NONE,
        AUTO,
        PHASE_1,
        PHASE_2,
        PHASE_3,
        PHASE_4,
        ENDGAME
    }
    //     public boolean isHubActive() {
    //   Optional<Alliance> alliance = DriverStation.getAlliance();
    //   // If we have no alliance, we cannot be enabled, therefore no hub.
    //   if (alliance.isEmpty()) {
    //     return false;
    //   }
    //   // Hub is always enabled in autonomous.
    //   if (DriverStation.isAutonomousEnabled()) {
    //     return true;
    //   }
    //   // At this point, if we're not teleop enabled, there is no hub.
    //   if (!DriverStation.isTeleopEnabled()) {
    //     return false;
    //   }

    //   // We're teleop enabled, compute.
    //   double matchTime = DriverStation.getMatchTime();
    //   String gameData = DriverStation.getGameSpecificMessage();
    //   // If we have no game data, we cannot compute, assume hub is active, as its likely early in
    // teleop.
    //   if (gameData.isEmpty()) {
    //     return true;
    //   }
    //   boolean redInactiveFirst = false;
    //   switch (gameData.charAt(0)) {
    //     case 'R' -> redInactiveFirst = true;
    //     case 'B' -> redInactiveFirst = false;
    //     default -> {
    //       // If we have invalid game data, assume hub is active.
    //       return true;
    //     }
    //   }

    //   // phase was is active for blue if red won auto, or red if blue won auto.
    //   boolean phase1Active = switch (alliance.get()) {
    //     case Red -> !redInactiveFirst;
    //     case Blue -> redInactiveFirst;
    //   };
    @AutoLogOutput(key = "Timing/RemainingShotTime")
    public static double remainingShotTime;

    @AutoLogOutput(key = "Timing/TimeUntilShot")
    public static double timeUntilShot;

    @AutoLogOutput(key = "Timing/MatchPhase")
    public static MatchPhase matchPhaseState = MatchPhase.NONE;

    public void matchPhaseUtil() {
        double matchTime = DriverStation.getMatchTime();
        double robotTime = Timer.getTimestamp();

        if (matchTime == lastMatchTime) {
            remainingTime += robotTime - lastRobotTime;
        } else {
            remainingTime = matchTime;
        }

        lastRobotTime = robotTime;
        lastMatchTime = matchTime;

        // if the timer -1, not in real match, so always allow shoot
        if (matchTime == -1) {
            remainingShotTime = 999;
            timeUntilShot = 0;
            return;
        }

        boolean wonAuton = wonAutonQuestion();

        if (remainingTime > 130) {
            // Transition phase, hub is active.
            if (wonAuton) {
                remainingShotTime = remainingTime - 130;
            } else {
                remainingShotTime = remainingTime - 105;
            }
            timeUntilShot = remainingTime - 160;

        } else if (remainingTime > 105) {
            // phase 1
            if (wonAuton) {
                remainingShotTime = remainingTime - 130;
                timeUntilShot = remainingTime - 105;
            } else {
                remainingShotTime = remainingTime - 105;
                timeUntilShot = remainingTime - 160;
            }
            MatchPhaseUtil.matchPhaseState = MatchPhase.PHASE_1;
        } else if (remainingTime > 80) {
            // phase 2
            if (wonAuton) {
                remainingShotTime = remainingTime - 80;
                timeUntilShot = remainingTime - 105;
            } else {
                remainingShotTime = remainingTime - 105;
                timeUntilShot = remainingTime - 80;
            }
            MatchPhaseUtil.matchPhaseState = MatchPhase.PHASE_2;
        } else if (remainingTime > 55) {
            // phase 3
            if (wonAuton) {
                remainingShotTime = remainingTime - 80;
                timeUntilShot = remainingTime - 55;
            } else {
                remainingShotTime = remainingTime - 55;
                timeUntilShot = remainingTime - 80;
            }
            MatchPhaseUtil.matchPhaseState = MatchPhase.PHASE_3;
        } else if (remainingTime > 30) {
            // phase 4
            if (wonAuton) {
                remainingShotTime = remainingTime;
                timeUntilShot = remainingTime - 55;
            } else {
                remainingShotTime = remainingTime - 55;
                timeUntilShot = remainingTime - 30;
            }
            MatchPhaseUtil.matchPhaseState = MatchPhase.PHASE_4;
        } else {
            // End game, hub always active.
            remainingShotTime = Math.max(remainingTime, 1);
            if (wonAuton) {
                timeUntilShot = remainingTime - 30;
            } else {
                timeUntilShot = remainingTime - 55;
            }
            MatchPhaseUtil.matchPhaseState = MatchPhase.ENDGAME;
        }
    }

    private boolean wonAutonQuestion() {
        String gameData = DriverStation.getGameSpecificMessage();
        boolean theBoolean = false;
        if (gameData.isBlank()) {
            return theBoolean;
        }
        if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
            switch (gameData.charAt(0)) {
                case 'R' -> theBoolean = true;
                case 'B' -> theBoolean = false;
            }
            return theBoolean;
        } else {
            switch (gameData.charAt(0)) {
                case 'R' -> theBoolean = false;
                case 'B' -> theBoolean = true;
            }
            return theBoolean;
        }
    }
}
