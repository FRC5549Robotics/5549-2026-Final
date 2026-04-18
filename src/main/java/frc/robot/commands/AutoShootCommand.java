package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
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

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;


public class AutoShootCommand extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final Limelight limelight;
    private final Shooter shooter;
    private final Hood hood;
    private final Belt belt;
    private final GroundIntake intake;

    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

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

    boolean waitingForPipeline = false;

    private final PIDController rotationPID = new PIDController(5, 0.0, 0.005);

    public AutoShootCommand(CommandSwerveDrivetrain drivetrain, Limelight limelight, Shooter shooter, Hood hood, Belt belt, GroundIntake intake) {
        this.drivetrain = drivetrain;
        this.limelight = limelight;
        this.shooter = shooter;
        this.hood = hood;
        this.belt = belt;
        this.intake = intake;

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

        //SmartDashboard.putString("state", state.toString());

        if (state == State.ALIGNING) {
            shooter.shoot(1500); //spin up to 1500
            Translation2d robot = drivetrain.getState().Pose.getTranslation();
            Translation2d target = Constants.HUB.get();

            Rotation2d direction = target.minus(robot).getAngle();
            double distance = robot.getDistance(target);

            ShooterState shot = ShooterLookup.get(distance);
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

            if (Math.abs(angleError) < Units.degreesToRadians(0.25) || Math.abs(angleError) > Units.degreesToRadians(359.75)) {
                state = State.SPINNING_UP;
            }
        }

       if (state == State.SPINNING_UP) {
            drivetrain.stopDriving();

            Translation2d robot = drivetrain.getState().Pose.getTranslation();
            Translation2d target = Constants.HUB.get();

            double distance = robot.getDistance(target);

           //SmartDashboard.putNumber("Distance", distance);

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
           if (!shooter.atSpeed()) {
               belt.off();
           }

           belt.intake();

            if (shootTimer.hasElapsed(3.5)) {
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
       shootTimer.stop();
       shootTimer.reset();

       second = false;
       waitingForPipeline = false;

       alignTimer.stop();
        alignTimer.reset();

        rotationPID.reset();
   }
}