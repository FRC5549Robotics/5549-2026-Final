package frc.robot.commands;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.SlewRateLimiter;
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
import edu.wpi.first.math.kinematics.ChassisSpeeds;


public class PassCommand extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final Shooter shooter;
    private final Hood hood;
    private final Belt belt;

    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    private final double MaxPassSpeed = 2.0;
    private final double MaxPassAccel = 2.5;
    private final SlewRateLimiter xLimiter = new SlewRateLimiter(MaxPassAccel);
    private final SlewRateLimiter yLimiter = new SlewRateLimiter(MaxPassAccel);
    private final PIDController rotationPID = new PIDController(2.65, 0.0, 0.0);

    private final Timer shootTimer = new Timer();
        private final Timer jamTimer = new Timer();

    public PassCommand(CommandSwerveDrivetrain drivetrain, Shooter shooter, Hood hood, Belt belt) {
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

    boolean beltUnjamming;

   @Override
   public void execute() {

        shootTimer.start();

        Translation2d robot = drivetrain.getState().Pose.getTranslation();
        Translation2d targetLeft = Constants.LeftPass.get();
        Translation2d targetRight = Constants.RightPass.get();

        Translation2d rawTarget = robot.getDistance(targetLeft) < robot.getDistance(targetRight) ? targetLeft : targetRight;
        
        double rawDistance = robot.getDistance(rawTarget);
        SmartDashboard.putNumber("pass distance", rawDistance);

        ShooterState shot = PassingLookup.get(rawDistance);

        ChassisSpeeds robotRelative = drivetrain.getChassisSpeeds();
        ChassisSpeeds fieldRelative = ChassisSpeeds.fromRobotRelativeSpeeds(
            robotRelative.vxMetersPerSecond, 
            robotRelative.vyMetersPerSecond, 
            robotRelative.omegaRadiansPerSecond,
            drivetrain.getState().Pose.getRotation()
        );

        Translation2d velocityOffset = new Translation2d(fieldRelative.vxMetersPerSecond * shot.timeOfFlight, fieldRelative.vyMetersPerSecond * shot.timeOfFlight);
        //System.out.println(velocityOffset);

        Translation2d compensatedTarget = rawTarget.minus(velocityOffset);
        drivetrain.setPassTarget(compensatedTarget);

        double compensatedDistance = robot.getDistance(compensatedTarget);
        Rotation2d direction = compensatedTarget.minus(robot).getAngle();
        ShooterState shotCompensated = PassingLookup.get(compensatedDistance);

        hood.setAngle(shotCompensated.hoodAngleDeg); //align hood while aligning drivetrain
        shooter.shoot(shotCompensated.flywheelRPM); //spin up while aligning

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
        double desiredX = MaxPassSpeed * -joystick.getLeftY();
        double desiredY = MaxPassSpeed * -joystick.getLeftX();
        Translation2d desiredVelocity = new Translation2d(desiredX, desiredY);
        if(desiredVelocity.getNorm() > MaxPassSpeed){
            desiredVelocity = desiredVelocity.times(MaxPassSpeed / desiredVelocity.getNorm());

        }
        double limitedX = xLimiter.calculate(desiredVelocity.getX());
        double limitedY = yLimiter.calculate(desiredVelocity.getY());
        drivetrain.setControl(
            drive
                .withVelocityX(limitedX)
                .withVelocityY(limitedY)
                .withRotationalRate(omega)
        );

        //SmartDashboard.putNumber("AngleErrorDeg", Units.radiansToDegrees(angleError));
        //SmartDashboard.putNumber("OmegaCmd", omega);

        boolean aimed = Math.abs(angleError) < Units.degreesToRadians(30);
        boolean spunUp = shootTimer.get() > 0.2;

        SmartDashboard.putBoolean("passing aimed", aimed);
        SmartDashboard.putBoolean("Passing spunup", spunUp);

        if (spunUp) {
            if (!belt.isIndexerJammed() && !beltUnjamming) {
                belt.intake();
                jamTimer.reset();
            } else if (belt.isIndexerJammed()){
                belt.jammed();
                jamTimer.reset();
                jamTimer.start();
                beltUnjamming = true;
                
                
            }
            else if(jamTimer.get() > 0.4){
                    beltUnjamming = false;
                }
        } else {
            belt.off();
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
        rotationPID.reset();
        xLimiter.reset(0);
        yLimiter.reset(0);
        shootTimer.stop();
        shootTimer.reset();
        shootTimer.start();
        
        beltUnjamming = false;
   }
}