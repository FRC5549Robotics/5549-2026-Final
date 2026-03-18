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
    private int lockedTagID = -1;

    Boolean second = false;

    private enum State {
        ALIGNING,
        SPINNING_UP,
        SHOOTING,
        DONE
    }

    private State state = State.ALIGNING;
    private final Timer shootTimer = new Timer();

    private final Timer alignTimer = new Timer();

    private final Timer tagLostTimer = new Timer();

    boolean waitingForPipeline = false;

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
        //if (!LimelightHelpers.getTV("limelight")) {
           //txFilter.reset();
           //return Double.NaN;
        //}

       double rawTx = LimelightHelpers.getTX("limelight");

       return txFilter.calculate(rawTx);
    }


    @Override
    public void execute() {

        if (waitingForPipeline) {
            if (LimelightHelpers.getCurrentPipelineIndex("limelight") == lastPipeline) {
                waitingForPipeline = false; // The switch is confirmed
                txFilter.reset();
            } else {
                System.out.println("still waiting");
                return; // Still waiting
            }
        }   

        System.out.println(state);
        if (state == State.ALIGNING) {
            double rotationOutput = 0;

            boolean hasTarget = LimelightHelpers.getTV("limelight");
            int seenTag = (int) LimelightHelpers.getFiducialID("limelight");

            shooter.shoot(1000);

            if (hasTarget) {

                if (lockedTagID == -1) {
                   lockedTagID = seenTag;
               }
          
               if (seenTag == lockedTagID) {
                   tagLostTimer.restart();
               }
          
               else if (tagLostTimer.hasElapsed(1)) {
                   lockedTagID = seenTag;
                   tagLostTimer.restart();
               }

                int tagID = (int) LimelightHelpers.getFiducialID("limelight");

                int desiredPipeline = 0;

                switch (lockedTagID) {
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
                case 1:
                case 12:
                case 6:
                case 7:
                case 17:
                case 28:
                case 22:
                case 23:
                    desiredPipeline = 3;
                    break;
                };

               if (desiredPipeline != lastPipeline) {
                   LimelightHelpers.setPipelineIndex("limelight", desiredPipeline);

                    waitingForPipeline = true;

                   lastPipeline = desiredPipeline;

                   drivetrain.aimDrive(0, 0, 0);
                   return;
               }

                double tx = getFilteredTX();
                double kP = 0.005549;
                double kS = 0.35;
                double tolerance = 0.75;
                double omega = tx * -kP * MaxAngularRate;

                System.out.println(tx);

                if (hasTarget && !Double.isNaN(tx)) {
                    if (Math.abs(tx) > tolerance) {

                    omega += Math.copySign(kS, omega);
                    rotationOutput = omega;

                    alignTimer.stop();
                    alignTimer.reset();

                    } else {
                    omega = 0.0;

                if (!alignTimer.isRunning()) {
                    alignTimer.restart();
                }

                if (alignTimer.hasElapsed(0.15)) { // time inside tolerance
                    if (desiredPipeline != 3) {
                        state = State.SPINNING_UP;
                    }
                }
            }
               }
            } else {
               // If we see nothing, allow switching soon
               if (tagLostTimer.hasElapsed(1)) {
                   lockedTagID = -1;
               }
            }
            

            drivetrain.aimDrive(0.0, 0.0, rotationOutput);
            return;
        }

       if (state == State.SPINNING_UP) {
           second = true;
           drivetrain.stopDriving();


           double distance = limelight.getDistanceToTagMeters();
           SmartDashboard.putNumber("Distance", distance);


           if (distance <= 0) {
               return;
           }


           ShooterState shot = ShooterLookup.get(distance);
           hood.setAngle(shot.hoodAngleDeg);
           shooter.shoot(shot.flywheelRPM);


           if (shooter.atSpeed()) {


               if (!shootTimer.isRunning()) {
                   shootTimer.reset();
                   shootTimer.start();
               }
          
                if (shootTimer.hasElapsed(0.1)) {
                    shootTimer.restart();
                    state = State.SHOOTING;
                }
          
           } else {
               shootTimer.stop();
               shootTimer.reset();
           }
           return;
           }
        if (state == State.SHOOTING) {
           if (!shooter.atSpeed()) {
               belt.off();
           }

           belt.intake();

            if (shootTimer.hasElapsed(4)) {
                intake.shooting();
            }

            if (shootTimer.hasElapsed(15)) {
                belt.off();
                intake.off();
                shooter.off();
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


       lockedTagID = -1;
       lastPipeline = -1;
       tagLostTimer.reset();
       tagLostTimer.start();

       second = false;
       waitingForPipeline = false;

       alignTimer.stop();
        alignTimer.reset();
   }
}