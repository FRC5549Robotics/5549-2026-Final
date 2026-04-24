package frc.robot;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Vision.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotStateEstimator extends SubsystemBase {
    private CommandSwerveDrivetrain m_SwerveDrivetrain;
    private boolean doRejectUpdate = false;
    private int visionCounter = 0;

    public RobotStateEstimator(CommandSwerveDrivetrain swerve) {
        m_SwerveDrivetrain = swerve;
                    m_SwerveDrivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(0.4,0.4,9999999));
    }

    @Override
    public void periodic() {
        if (++visionCounter < 5) {
            return;
        }
        visionCounter = 0;
        doRejectUpdate = false;
        //LimelightHelpers.SetRobotOrientation("limelight", m_SwerveDrivetrain.getPigeon2().getYaw().getValueAsDouble(), 0, 0, 0, 0, 0);
        LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
        if (mt1 == null) {
            return;
        }
        if (Math.abs(m_SwerveDrivetrain.getPigeon2().getAngularVelocityZWorld().getValueAsDouble()) > 720) {  //if angular velocity > 720 deg/s, ignore vision
            doRejectUpdate = true;
        } 
        if (mt1.tagCount == 0) {
            //System.out.println("tag count = 0");
            doRejectUpdate = true;
        }
        SmartDashboard.putNumber("avgTagDist", mt1.avgTagDist);
        if (mt1.avgTagDist > 2) {
            doRejectUpdate = true;
        }
        if (!doRejectUpdate) {
            //System.out.println("pulling vision");
            m_SwerveDrivetrain.addVisionMeasurement(mt1.pose, mt1.timestampSeconds);
        }
    }
}
