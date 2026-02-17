package frc.robot.shooter;

import java.util.Map;
import java.util.TreeMap;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.SerializationFeature;
import java.io.File;
import java.io.IOException;
import java.util.List;
import java.util.ArrayList;


public class ShooterLookup {

    // distance (meters) -> ShooterState
    private static final TreeMap<Double, ShooterState> table = new TreeMap<>();

    private static final int NUM_POINTS = 6; // how many data points
    private static boolean initialized = false;

    private static final File SAVE_FILE = new File("/home/lvuser/shooter_lookup.json");
    private static final ObjectMapper mapper = new ObjectMapper().enable(SerializationFeature.INDENT_OUTPUT);

    public static void initializeDashboard() {
        if (initialized) return;

        putDefault(0, 1.0, 78, 1860);
        putDefault(1, 1.5, 77, 1970);
        putDefault(2, 2.0, 76.5, 2115);
        putDefault(3, 2.5, 75.5, 2235);
        putDefault(4, 3.0, 75, 2275);
        putDefault(5, 3.5, 74, 2380);

        initialized = true;

        SmartDashboard.putBoolean("Save Shooter Table", false);
    }

    private static void putDefault(int i, double dist, double angle, double rpm) {
        SmartDashboard.putNumber("Shot" + i + "_Distance", dist);
        SmartDashboard.putNumber("Shot" + i + "_Angle", angle);
        SmartDashboard.putNumber("Shot" + i + "_RPM", rpm);
    }

    private static void rebuildTableFromDashboard() {
        table.clear();

        for (int i = 0; i < NUM_POINTS; i++) {
            double dist = SmartDashboard.getNumber("Shot" + i + "_Distance", 0);
            double angle = SmartDashboard.getNumber("Shot" + i + "_Angle", 0);
            double rpm = SmartDashboard.getNumber("Shot" + i + "_RPM", 0);

            table.put(dist, new ShooterState(angle, rpm));
        }
    }

    public static class ShotEntry {
        public double distance;
        public double angle;
        public double rpm;

        public ShotEntry() {}

        public ShotEntry(double d, double a, double r) {
            distance = d;
            angle = a;
            rpm = r;
        }
    }

    public static void saveToFile() {
        try {
            List<ShotEntry> entries = new ArrayList <>();

            for (int i = 0; i < NUM_POINTS; i++) {
                double dist = SmartDashboard.getNumber("Shot" + i + "_Distance", 0);
                double angle = SmartDashboard.getNumber("Shot" + i + "_Angle", 0);
                double rpm = SmartDashboard.getNumber("Shot" + i + "_RPM", 0);

                entries.add(new ShotEntry(dist, angle, rpm));
            }

            mapper.writeValue(SAVE_FILE, entries);
            System.out.println("Shooter table saved");
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public static void loadFromFile() {
        if (!SAVE_FILE.exists()) return;

        try {
            ShotEntry[] entries = mapper.readValue(SAVE_FILE, ShotEntry[].class);

            for (int i = 0; i < NUM_POINTS; i++) {
                SmartDashboard.putNumber("Shot" + i + "_Distance", entries[i].distance);
                SmartDashboard.putNumber("Shot" + i + "_Angle", entries[i].angle);
                SmartDashboard.putNumber("Shot" + i + "_RPM", entries[i].rpm);
            }

            System.out.println("Shooter table loaded");
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public static void checkForSaveCommand() {
        if (SmartDashboard.getBoolean("Save Shooter Table", false)) {
            saveToFile();
            SmartDashboard.putBoolean("Save Shooter Table", false);
        }
    }

    public static ShooterState get(double distanceMeters) {

        rebuildTableFromDashboard();

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