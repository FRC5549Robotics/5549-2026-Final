// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Vision.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AlignLimelight extends Command {
  private PIDController rotController;
  // xController, yController,
  private boolean isRightScore;
  private Timer dontSeeTagTimer, stopTimer;
  private CommandSwerveDrivetrain m_drive;
  private double tagID = -1;

  public AlignLimelight(boolean isRightScore, CommandSwerveDrivetrain m_drive) {
    rotController = new PIDController(Constants.ROT_HUB_ALIGNMENT_P, 0, 0);  // Rotation
    // xController = new PIDController(0.02, 0, 0); 
    // yController = new PIDController(0.02, 0, 0);
    this.isRightScore = isRightScore;
    this.m_drive = m_drive;
    addRequirements(m_drive);
  }

  @Override
  public void initialize() {
    this.stopTimer = new Timer();
    this.stopTimer.start();
    this.dontSeeTagTimer = new Timer();
    this.dontSeeTagTimer.start();

    rotController.setSetpoint(Constants.ROT_SETPOINT_HUB_ALIGNMENT);
    rotController.setTolerance(Constants.ROT_TOLERANCE_HUB_ALIGNMENT);

    // xController.setSetpoint(0); 
    // yController.setSetpoint(0);
    
    // You must set tolerance, otherwise .atSetpoint() will always be false (or true depending on defaults)
    // xController.setTolerance(Constants.TRANSLATION_TOLERANCE); 
    // yController.setTolerance(Constants.TRANSLATION_TOLERANCE);

    tagID = LimelightHelpers.getFiducialID("");
    System.out.println("pls initialize limelight thank you");
  }

  @Override
  public void execute() {
    System.out.println("Aligning?");
    if (LimelightHelpers.getTV("limelight") && LimelightHelpers.getFiducialID("limelight") == tagID) {
      this.dontSeeTagTimer.reset();

      double[] positions = LimelightHelpers.getBotPose_TargetSpace("");
      SmartDashboard.putNumber("x", positions[2]);

      // double xSpeed = xController.calculate(postions[2]);
      // SmartDashboard.putNumber("xspeed", xSpeed);
      // double ySpeed = -yController.calculate(postions[0]);
      double rotValue = -rotController.calculate(positions[4]);

      m_drive.setControl(
        new SwerveRequest.FieldCentric()
        // .withVelocityX(xSpeed)
        // .withVelocityY(ySpeed)
        .withRotationalRate(rotValue)
        );


      if (!rotController.atSetpoint() ) {
        // ||
        //   !yController.atSetpoint() ||
        //   !xController.atSetpoint()
        stopTimer.reset();
      }
    } else {
      m_drive.setControl(
    new SwerveRequest.FieldCentric()
        .withVelocityX(0)
        .withVelocityY(0)
        .withRotationalRate(0)
);

    }

    SmartDashboard.putNumber("poseValidTimer", stopTimer.get());
  }

  @Override
  public void end(boolean interrupted) {
    m_drive.setControl(
    new SwerveRequest.FieldCentric()
        .withVelocityX(0)
        .withVelocityY(0)
        .withRotationalRate(0)
);

  }

  @Override
  public boolean isFinished() {
    // Requires the robot to stay in the correct position for 0.3 seconds, as long as it gets a tag in the camera
    return this.dontSeeTagTimer.hasElapsed(Constants.DONT_SEE_TAG_WAIT_TIME) ||
        stopTimer.hasElapsed(Constants.POSE_VALIDATION_TIME);
  }
}