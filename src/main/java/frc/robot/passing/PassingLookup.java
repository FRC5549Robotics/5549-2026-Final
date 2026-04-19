package frc.robot.passing;

import java.util.Map;
import java.util.TreeMap;

import frc.robot.shooter.ShooterState;

import edu.wpi.first.wpilibj.Preferences;

public class PassingLookup {

    // distance (meters) -> ShooterState
    private static final TreeMap<Double, ShooterState> table = new TreeMap<>();

    public static void updateTableFromPreferences() {
        table.clear();

        for (int i = 1; i <= 3; i++) {
            double dist = Preferences.getDouble("Passing_Dist_" + i, 0.0);
            double angle = Preferences.getDouble("Passing_Angle_" + i, 0.0);
            double rpm = Preferences.getDouble("Passing_RPM_" + i, 0.0);
            double tof = Preferences.getDouble("Passing_Time_" + i, 0.0);

            if (dist > 0) {
                table.put(dist, new ShooterState(angle, rpm, tof));
            }
        }

    }

    public static ShooterState get(double distanceMeters) {

        if (table.isEmpty()) return new ShooterState(72, 0,0);

        Map.Entry<Double, ShooterState> lower = table.floorEntry(distanceMeters);
        Map.Entry<Double, ShooterState> upper = table.ceilingEntry(distanceMeters);

        if (lower == null) return upper.getValue();
        if (upper == null) return lower.getValue();
        if (lower.getKey().equals(upper.getKey())) return lower.getValue();

        double t = (distanceMeters - lower.getKey()) / (upper.getKey() - lower.getKey());

        return lower.getValue().interpolate(upper.getValue(), t);
    }
}