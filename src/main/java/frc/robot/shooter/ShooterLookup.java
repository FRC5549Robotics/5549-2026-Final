package frc.robot.shooter;

import java.util.Map;
import java.util.TreeMap;

import edu.wpi.first.wpilibj.Preferences;

public class ShooterLookup {

    // distance (meters) -> ShooterState
    private static final TreeMap<Double, ShooterState> table = new TreeMap<>();

    public static void updateTableFromPreferences() {
        table.clear();

        for (int i = 1; i <= 6; i++) {
            double dist = Preferences.getDouble("Shooter_Dist_" + i, 0.0);
            double angle = Preferences.getDouble("Shooter_Angle_" + i, 0.0);
            double rpm = Preferences.getDouble("Shooter_RPM_" + i, 0.0);

            if (dist > 0) {
                table.put(dist, new ShooterState(angle, rpm));
            }
        }

    }

    // static {
    //     table.put(1.043158, new ShooterState(78.5, 1721));
    //     table.put(1.455167, new ShooterState(77.5, 1883));
    //     table.put(2.038916, new ShooterState(74, 2035));
    //     table.put(2.415026, new ShooterState(73, 2288));
    //     table.put(2.944653, new ShooterState(72, 2450));
    //     table.put(3.383125, new ShooterState(71, 2633));
    // }

    public static ShooterState get(double distanceMeters) {

        if (table.isEmpty()) return new ShooterState(72, 0);

        Map.Entry<Double, ShooterState> lower = table.floorEntry(distanceMeters);
        Map.Entry<Double, ShooterState> upper = table.ceilingEntry(distanceMeters);

        if (lower == null) return upper.getValue();
        if (upper == null) return lower.getValue();
        if (lower.getKey().equals(upper.getKey())) return lower.getValue();

        double t = (distanceMeters - lower.getKey()) / (upper.getKey() - lower.getKey());

        return lower.getValue().interpolate(upper.getValue(), t);
    }
}