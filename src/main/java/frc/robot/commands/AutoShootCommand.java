package frc.robot.commands;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.Belt;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.GroundIntake;

import frc.robot.Vision.LimelightHelpers;
import frc.robot.shooter.ShooterLookup;
import frc.robot.shooter.ShooterState;

import static edu.wpi.first.units.Units.*;


public class AutoShootCommand extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final Limelight limelight;
    private final Shooter shooter;
    private final Hood hood;
    private final Belt belt;
    private final GroundIntake intake;

    private final LinearFilter txFilter = LinearFilter.movingAverage(4);

    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    private int lastPipeline = -1;

    private enum State {
        ALIGNING,
        SPINNING_UP,
        SHOOTING,
        DONE
    }

    private State state = State.ALIGNING;
    private final Timer shootTimer = new Timer();

    public AutoShootCommand(CommandSwerveDrivetrain drivetrain, Limelight limelight, Shooter shooter, Hood hood, Belt belt, GroundIntake intake) {
        this.drivetrain = drivetrain;
        this.limelight = limelight;
        this.shooter = shooter;
        this.hood = hood;
        this.belt = belt;
        this.intake = intake;

        addRequirements(drivetrain, shooter, hood, belt, intake);
    }

    public double getFilteredTX() {
        System.out.println("autoing");
        if (!LimelightHelpers.getTV("limelight")) {
            txFilter.reset();
            return Double.NaN;
        }

        double rawTx = LimelightHelpers.getTX("limelight");

        return txFilter.calculate(rawTx);
    }

    @Override
    public void execute() {
        System.out.println(state);
        if (state == State.ALIGNING) {
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

                if (desiredPipeline != lastPipeline) {
                    LimelightHelpers.setPipelineIndex("limelight", desiredPipeline);
                    lastPipeline = desiredPipeline;
                }

                double tx = getFilteredTX();
                double kP = 0.015;

                if (!Double.isNaN(tx)) {
                    rotationOutput = tx * -kP * MaxAngularRate;

                    if (Math.abs(tx) < 1.0) {
                        state = State.SPINNING_UP;
                    }
                }
            }

            drivetrain.aimDrive(0.0, 0.0, rotationOutput);
            return;
        }

        if (state == State.SPINNING_UP) {
            drivetrain.stopDriving();

            double distance = limelight.getDistanceToTagMeters();

            if (distance <= 0) {
                return;
            }

            ShooterState shot = ShooterLookup.get(distance);
            hood.setAngle(shot.hoodAngleDeg);
            shooter.shoot(shot.flywheelRPM);

            if (shooter.atSpeed() && hood.atTarget()) {
                shootTimer.reset();
                shootTimer.start();
                state = State.SHOOTING;
            }
            return;
        }
        if (state == State.SHOOTING) {
            belt.intake();

            if (shootTimer.hasElapsed(1)) {
                intake.shooting();
            }

            if (shootTimer.hasElapsed(7)) {
                belt.off();
                intake.off();
                state = State.DONE;
            }
            return;
        }
    }

    @Override
    public void end(boolean interupted) {
        shooter.off();
        belt.off();
        intake.off();
        drivetrain.stopDriving();
    }

    @Override
    public boolean isFinished() {
        return state == State.DONE;
    }

    @Override
    public void initialize() {
        state = State.ALIGNING;
        txFilter.reset();
        shootTimer.stop();
        shootTimer.reset();
    }
}