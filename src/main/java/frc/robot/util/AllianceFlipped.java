package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import java.util.function.Supplier;

public class AllianceFlipped<T> {

        private final Supplier<T> blueSupplier;
        private final Supplier<T> redSupplier;

        public AllianceFlipped(Supplier<T> blue, Supplier <T> red) {
            this.blueSupplier = blue;
            this.redSupplier = red;
        }

        public T get() {
            return DriverStation.getAlliance()
                .orElse(Alliance.Blue) == Alliance.Red
                ? redSupplier.get()
                : blueSupplier.get();
        }
}
