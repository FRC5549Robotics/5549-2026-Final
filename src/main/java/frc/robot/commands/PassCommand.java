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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.XboxController;

import frc.robot.subsystems.Belt;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import pabeles.concurrency.IntOperatorTask.Max;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.GroundIntake;
import frc.robot.Constants;
import frc.robot.Vision.LimelightHelpers;
import frc.robot.passing.PassingLookup;
import frc.robot.shooter.ShooterLookup;
import frc.robot.shooter.ShooterState;

import static edu.wpi.first.units.Units.*;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

public class PassCommand extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final Shooter shooter;
    private final Hood hood;
    private final Belt belt;

    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    private enum State {
        AIMING_AND_SPINNING,
        PASSING
    }

    private State state = State.AIMING_AND_SPINNING;

    private final Timer shootTimer = new Timer();
    private final Timer alignTimer = new Timer();

    private final PIDController rotationPID = new PIDController(5, 0.0, 0.005);

    public PassCommand(CommandSwerveDrivetrain drivetrain, Limelight limelight, Shooter shooter, Hood hood, Belt belt, GroundIntake intake, BooleanSupplier allowAutoPivot) {
       this.drivetrain = drivetrain;
       this.shooter = shooter;
       this.hood = hood;
       this.belt = belt;

        rotationPID.enableContinuousInput(-Math.PI, Math.PI);

       addRequirements(drivetrain, shooter, belt);
   }
    
   private final double MaxSpeed = 4.5;

   private SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(MaxSpeed * 0.1)
        .withRotationalDeadband(MaxAngularRate * 0.1)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

   private XboxController joystick = new XboxController(0);

   @Override
   public void execute() {

        //System.out.println(state);
        if (state == State.AIMING_AND_SPINNING) {
            Translation2d robot = drivetrain.getState().Pose.getTranslation();
            Translation2d targetLeft = Constants.LeftPass.get();
            Translation2d targetRight = Constants.RightPass.get();

            Rotation2d direction;
            double distance;

            if (robot.getDistance(targetLeft) < robot.getDistance(targetRight)) {
                direction = targetLeft.minus(robot).getAngle();
                distance = robot.getDistance(targetLeft);
            } else {
                direction = targetRight.minus(robot).getAngle();
                distance = robot.getDistance(targetRight);
            }

            ShooterState shot = PassingLookup.get(distance);
            hood.setAngle(shot.hoodAngleDeg); //align hood while aligning drivetrain
            shooter.shoot(shot.flywheelRPM); //spin up while aligning

            double currentAngle = drivetrain.getState().Pose.getRotation().getRadians();
            double targetAngle = direction.getRadians();

            double omega = rotationPID.calculate(currentAngle + Math.PI, targetAngle);

            double angleError = MathUtil.angleModulus(targetAngle - currentAngle);

            angleError = angleError + Math.PI;

            double kS = 0.45;

            if (Math.abs(angleError) < Units.degreesToRadians(10) || Math.abs(angleError) > Units.degreesToRadians(350)) {
                omega += Math.copySign(kS, omega);
            }

            omega = MathUtil.clamp(omega, -MaxAngularRate, MaxAngularRate);

            drivetrain.setControl(
                drive
                    .withVelocityX(MaxSpeed * -joystick.getLeftY())
                    .withVelocityY(MaxSpeed * -joystick.getLeftX())
                    .withRotationalRate(omega)
            );

            //SmartDashboard.putNumber("AngleErrorDeg", Units.radiansToDegrees(angleError));
            //SmartDashboard.putNumber("OmegaCmd", omega);

            if ((Math.abs(angleError) < Units.degreesToRadians(0.25) || Math.abs(angleError) > Units.degreesToRadians(359.75)) && shooter.atSpeed()) {
                 if (!shootTimer.isRunning()) {
                   shootTimer.reset();
                   shootTimer.start();
               }
          
               if (shootTimer.hasElapsed(0.05)) { //wait a short time to ensure rpm is steady
                   state = State.PASSING;
               }
           } else {
               shootTimer.stop();
               shootTimer.reset();
           }
           return;
        }

       if (state == State.PASSING) {

           if (!shooter.atSpeed()) {
               belt.off();
               return;
           }

           
            Translation2d robot = drivetrain.getState().Pose.getTranslation();
            Translation2d targetLeft = Constants.LeftPass.get();
            Translation2d targetRight = Constants.RightPass.get();

            Rotation2d direction;

            if (robot.getDistance(targetLeft) < robot.getDistance(targetRight)) {
                direction = targetLeft.minus(robot).getAngle();
            } else {
                direction = targetRight.minus(robot).getAngle();
            }

            double currentAngle = drivetrain.getState().Pose.getRotation().getRadians();
            double targetAngle = direction.getRadians();

            double angleError = MathUtil.angleModulus(targetAngle - currentAngle);

            if (Math.abs(angleError) > Units.degreesToRadians(0.25) && Math.abs(angleError) < Units.degreesToRadians(359.75)) {
                state = State.AIMING_AND_SPINNING;
                return;
            }

            belt.intake();

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
       state = State.AIMING_AND_SPINNING;
       shootTimer.stop();
       shootTimer.reset();

        alignTimer.stop();
        alignTimer.reset();

        rotationPID.reset();
   }
}