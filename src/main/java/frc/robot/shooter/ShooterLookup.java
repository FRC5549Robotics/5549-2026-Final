package frc.robot.shooter;

import java.util.Map;
import java.util.TreeMap;

public class ShooterLookup {

// distance (meters) -> ShooterState
private static final TreeMap<Double, ShooterState> table = new TreeMap<>();

static {
    table.put(1.012623, new ShooterState(78.5, 820));
    table.put(1.443128, new ShooterState(77.5, 900));
    table.put(1.936055, new ShooterState(76.2, 1000));
    table.put(2.415026, new ShooterState(75.5, 1100));
    table.put(2.944653, new ShooterState(74.5, 1180));
    table.put(3.383125, new ShooterState(74, 1270));
}

public static ShooterState get(double distanceMeters) {

Map.Entry<Double, ShooterState> lower = table.floorEntry(distanceMeters);
Map.Entry<Double, ShooterState> upper = table.ceilingEntry(distanceMeters);

if (lower == null) return upper.getValue();
if (upper == null) return lower.getValue();
if (lower.getKey().equals(upper.getKey())) return lower.getValue();

double t =
(distanceMeters - lower.getKey()) /
(upper.getKey() - lower.getKey());

return lower.getValue().interpolate(upper.getValue(), t);
}
}