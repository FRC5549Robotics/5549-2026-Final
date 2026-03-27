package frc.robot.util;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class AllianceFlippedUtil {
    
    public static final double FIELD_LENGTH = Units.inchesToMeters(651.2);

    public static AllianceFlipped<Translation2d> fromBlue (Translation2d blue) {
        return new AllianceFlipped<>(
            () -> blue,
            () -> new Translation2d(
                FIELD_LENGTH - blue.getX(),
                blue.getY()
            )
        );
    }
}
