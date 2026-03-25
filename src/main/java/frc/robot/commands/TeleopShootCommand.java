package frc.robot.commands;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import frc.robot.subsystems.Belt;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.GroundIntake;
import frc.robot.Constants;
import frc.robot.Vision.LimelightHelpers;
import frc.robot.shooter.ShooterLookup;
import frc.robot.shooter.ShooterState;

import static edu.wpi.first.units.Units.*;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

public class TeleopShootCommand extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final Limelight limelight;
    private final Shooter shooter;
    private final Hood hood;
    private final Belt belt;
    private final GroundIntake intake;

    private final BooleanSupplier allowAutoPivot;

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    private enum State {
       ALIGNING,
       SPINNING_UP,
       SHOOTING
    }

   private State state = State.ALIGNING;

   private final Timer shootTimer = new Timer();
    private final Timer alignTimer = new Timer();

   private final PIDController rotationPID = new PIDController(4.0, 0.0, 0.3);

   public TeleopShootCommand(CommandSwerveDrivetrain drivetrain, Limelight limelight, Shooter shooter, Hood hood, Belt belt, GroundIntake intake, BooleanSupplier allowAutoPivot) {
       this.drivetrain = drivetrain;
       this.limelight = limelight;
       this.shooter = shooter;
       this.hood = hood;


       this.belt = belt;
       this.intake = intake;
       this.allowAutoPivot = allowAutoPivot;

        rotationPID.enableContinuousInput(-Math.PI, Math.PI);

       addRequirements(drivetrain, shooter, hood, belt);
   }

    private Pose2d getTargetPose() {
        var alliance = DriverStation.getAlliance();

        if (alliance.isPresent() && alliance.get() == Alliance.Red) {
            return Constants.RED_HUB;
        } else {
            return Constants.BLUE_HUB;
        }
    }

   @Override
   public void execute() {

        System.out.println(state);
        if (state == State.ALIGNING) {
            Pose2d robotpose = drivetrain.getPose();
            Pose2d targetPose = getTargetPose();

            Translation2d delta = targetPose.getTranslation().minus(robotpose.getTranslation());

            double targetAngle = Math.atan2(delta.getY(), delta.getX());

            double currentAngle = robotpose.getRotation().getRadians();

            double omega = rotationPID.calculate(currentAngle, targetAngle);

            omega = Math.max(Math.min(omega, MaxAngularRate), -MaxAngularRate);

            if (Math.abs(omega) > 0.01) {
                omega += Math.copySign(0.35, omega);
            }

            double angleError = Math.abs(targetAngle - currentAngle);

            angleError = MathUtil.angleModulus(angleError);

            if (Math.abs(angleError) < Units.degreesToRadians(1)) {

                if (!alignTimer.isRunning()) {
                    alignTimer.restart();
                }

                if (alignTimer.hasElapsed(0.15)) {
                    state = State.SPINNING_UP;
                }

                omega = 0;

            } else {
                alignTimer.stop();
                alignTimer.reset();
            }

            drivetrain.aimDrive(0.0, 0.0, omega);

            return;
        }


       if (state == State.SPINNING_UP || state == State.SHOOTING) {
           drivetrain.setControl(brake);
       }

       if (state == State.SPINNING_UP) {
           drivetrain.stopDriving();

            Pose2d robotPose = drivetrain.getPose();
            Pose2d targetPose = getTargetPose();

            double distance = robotPose.getTranslation().getDistance(targetPose.getTranslation());

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
          
               if (shootTimer.hasElapsed(0.2)) {
                   state = State.SHOOTING;
               }
          
           } else {
               shootTimer.stop();
               shootTimer.reset();
           }
           return;
           }
       if (state == State.SHOOTING) {
           //if (Math.abs(getFilteredTX()) > 5.0) { //check if it goes out of tolerance
               //belt.off();
               //intake.off();
               //state = State.ALIGNING;
           //}


           if (!shooter.atSpeed()) {
               belt.off();
           }


           belt.intake();


           //if (shootTimer.hasElapsed(2.5)) { //delay before the intake goes up and down
               //if (allowAutoPivot.getAsBoolean()) {
                  // intake.shooting();
               //}
           //}


           return;
       }
   }


   @Override
   public void end(boolean interupted) {
       shooter.off();
       belt.off();
       //if (allowAutoPivot.getAsBoolean()) {
           //intake.shooting();
       //
      
  
   //}
       drivetrain.stopDriving();
   }


   @Override
   public boolean isFinished() {
       return false;
   }


   @Override
   public void initialize() {
       state = State.ALIGNING;
       shootTimer.stop();
       shootTimer.reset();

        alignTimer.stop();
        alignTimer.reset();
   }
}