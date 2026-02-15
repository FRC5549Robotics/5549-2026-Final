package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.shooter.ShooterLookup;
import frc.robot.shooter.ShooterState;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;

public class AimAndSpinUpCommand extends Command {
    private final Limelight limelight;
    private final Shooter shooter;
    private final Hood hood;

    public AimAndSpinUpCommand(Limelight limelight, Shooter shooter, Hood hood) {
        this.limelight = limelight;
        this.shooter = shooter;
        this.hood = hood;
        addRequirements(shooter, hood);
    }

    @Override
    public void execute() {
        double distance = limelight.getDistanceToTagMeters();
        SmartDashboard.putNumber("Distance", distance);

        if (distance <= 0) {
            return;
        }

        ShooterState shot = ShooterLookup.get(distance);
        //SmartDashboard.putNumber("Target Pos", shot.hoodAngleDeg);
        hood.setAngle(shot.hoodAngleDeg);
        shooter.shoot1(shot.flywheelRPM);
        SmartDashboard.putNumber("flywheelRPM", shot.flywheelRPM);
    }
}


