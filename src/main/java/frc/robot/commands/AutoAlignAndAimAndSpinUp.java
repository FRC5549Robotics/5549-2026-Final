package frc.robot.commands;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Hood;

import frc.robot.Constants;

import frc.robot.Vision.LimelightHelpers;
import frc.robot.shooter.ShooterLookup;
import frc.robot.shooter.ShooterState;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveRequest;


public class AutoAlignAndAimAndSpinUp extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final Limelight limelight;
    private final Shooter shooter;
    private final Hood hood;

    private final LinearFilter txFilter = LinearFilter.movingAverage(4);

    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    public AutoAlignAndAimAndSpinUp(CommandSwerveDrivetrain drivetrain, Limelight limelight, Shooter shooter, Hood hood) {
        this.drivetrain = drivetrain;
        this.limelight = limelight;
        this.shooter = shooter;
        this.hood = hood;

        addRequirements(drivetrain, shooter, hood);
    }

    public double getFilteredTX() {
        if (!LimelightHelpers.getTV("limelight")) {
            txFilter.reset();
            return Double.NaN;
        }

        double rawTx = LimelightHelpers.getTX("limelight");

        return txFilter.calculate(rawTx);
    }

    @Override
    public void execute() {
        double rotationOutput = 0;

        boolean hasTarget = LimelightHelpers.getTV("limelight");

        if (hasTarget) {

            int tagID = (int) LimelightHelpers.getFiducialID("limelight");

            int desiredPipeline = 0;

            switch (tagID) {
            // middle
            case 5:
            case 10:
            case 2:
            case 18:
            case 26:
            case 21:
                desiredPipeline = 0;
                break;
            // right
            case 8:
            case 24:
                desiredPipeline = 1;
                break;
            //left
            case 9:
            case 11:
            case 25:
            case 27:
                desiredPipeline = 2;
                break;
            };

            LimelightHelpers.setPipelineIndex("limelight", desiredPipeline);

            double tx = getFilteredTX();
            double kP = 0.015;

            if (!Double.isNaN(tx)) {
                rotationOutput = tx * -kP * MaxAngularRate;
            }
        }

        drivetrain.aimDrive(0.0, 0.0, rotationOutput);

        double distance = limelight.getDistanceToTagMeters();

        if (distance <= 0) {
            return;
        }

        ShooterState shot = ShooterLookup.get(distance);
        hood.setAngle(shot.hoodAngleDeg);
        shooter.shoot(shot.flywheelRPM);
    }

    @Override
    public void end(boolean interupted) {
        shooter.off();
        drivetrain.stopDriving();
    }

    @Override
    public boolean isFinished() {
        double tx = LimelightHelpers.getTX("limelight");
        return Math.abs(tx) < 5;
    }
}
