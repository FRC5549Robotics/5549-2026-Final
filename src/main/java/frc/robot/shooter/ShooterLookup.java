package frc.robot.shooter;

import java.util.Map;
import java.util.TreeMap;

public class ShooterLookup {

// distance (meters) -> ShooterState
private static final TreeMap<Double, ShooterState> table = new TreeMap<>();

static {
table.put(1.046011, new ShooterState(78, 1860));
table.put(1.5, new ShooterState(77.5, 1970));
table.put(1.807899, new ShooterState(77, 2030));
table.put(2.0, new ShooterState(76.5,2115));
table.put(2.592646, new ShooterState(76, 2285));
table.put(2.956945, new ShooterState(75.7, 2320));
table.put(3.5, new ShooterState(74, 2380));
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