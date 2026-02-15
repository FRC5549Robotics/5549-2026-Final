package frc.robot.shooter;

import java.util.Map;
import java.util.TreeMap;

public class ShooterLookup {

// distance (meters) -> ShooterState
private static final TreeMap<Double, ShooterState> table = new TreeMap<>();

static {
table.put(1.5, new ShooterState(70, 50));
table.put(2.0, new ShooterState(72, 100));
table.put(2.5, new ShooterState(74, 150));
table.put(3.0, new ShooterState(76, 200));
table.put(3.5, new ShooterState(78, 250));
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