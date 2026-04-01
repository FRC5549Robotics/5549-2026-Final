package frc.robot.shooter;

import edu.wpi.first.math.interpolation.Interpolatable;

public class ShooterState implements Interpolatable<ShooterState> {

    public final double hoodAngleDeg;
    public final double flywheelRPM;
    public final double timeOfFlight;

    public ShooterState(double hoodAngleDeg, double flywheelRPM, double timeOfFlight) {
        this.hoodAngleDeg = hoodAngleDeg;
        this.flywheelRPM = flywheelRPM;
        this.timeOfFlight = timeOfFlight;
    }

    /**
     * This is called automatically by WPILib when we request a distance
     * between two known calibration points.
     */
    @Override
    public ShooterState interpolate(ShooterState endValue, double t) {

        double newAngle = hoodAngleDeg + (endValue.hoodAngleDeg - hoodAngleDeg) * t;

        double newRPM = flywheelRPM + (endValue.flywheelRPM - flywheelRPM) * t;

        double newTOF = timeOfFlight + (endValue.timeOfFlight - timeOfFlight) * t;

        return new ShooterState(newAngle, newRPM, newTOF);
    }
}
