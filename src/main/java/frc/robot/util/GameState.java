package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import java.util.Optional;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class GameState {
    
    private boolean redInactiveFirst = false;
    private boolean hasGameData = false;

    public void update() {
        if (hasGameData) return;

        String msg = DriverStation.getGameSpecificMessage();
        if (!msg.isEmpty()) {
            redInactiveFirst = msg.
            charAt(0) == 'R';
            hasGameData = true;
        }
    }

    public boolean isHubInactiveNow() {

        Optional<DriverStation.Alliance> allianceOpt = DriverStation.getAlliance();

        if (allianceOpt.isEmpty()) return false;

        if (DriverStation.isAutonomousEnabled()) return false; //not inactive during auto

        if (!DriverStation.isTeleopEnabled()) return true; //

        DriverStation.Alliance alliance = allianceOpt.get();

        double matchTime = DriverStation.getMatchTime();

        boolean shift1Active = (alliance == DriverStation.Alliance.Red) ? !redInactiveFirst : redInactiveFirst; //I checked, it's getting the correct shift1Active

        //add 2 sec to everything to account for it taking time to get to the hub

        if (matchTime > 130) return false; //transition period, no hub is inactive no matter what
        else if (matchTime > 107) return !shift1Active; //shift one
        else if (matchTime > 82) return shift1Active; //shift 2
        else if (matchTime > 57) return !shift1Active; //shift 3
        else if (matchTime > 32) return shift1Active; //shift 4
        else return false; //endgame is always active

    }

    public double getSecondsUntilHubToggle() {

        if (!DriverStation.isTeleopEnabled()) return -1;

        double matchTime = DriverStation.getMatchTime();

        double[] transitions = {130,107,82,57,32}; 

        for (double t : transitions) {
            if (matchTime > t) {
                return matchTime - t;
            }
        }

        return -1;
    }
}