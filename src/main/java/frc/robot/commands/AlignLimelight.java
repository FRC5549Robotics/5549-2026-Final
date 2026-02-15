// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Vision.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AlignLimelight extends Command {
  private PIDController rotController;
  private boolean isRightScore;
  private Timer dontSeeTagTimer, stopTimer;
  private CommandSwerveDrivetrain m_drive;
  private double tagID = -1;

  // New: Suppliers for driving
  private DoubleSupplier xSupplier;
  private DoubleSupplier ySupplier;
  private boolean isDrivingMode = false;

  // CONSTRUCTOR 1: Stationary Align (Original)
  public AlignLimelight(boolean isRightScore, CommandSwerveDrivetrain m_drive) {
    this(isRightScore, m_drive, () -> 0.0, () -> 0.0);
    this.isDrivingMode = false; // Mark as stationary so timers work
  }

  // CONSTRUCTOR 2: Driving Align (New)
  public AlignLimelight(boolean isRightScore, CommandSwerveDrivetrain m_drive, DoubleSupplier x, DoubleSupplier y) {
    this.m_drive = m_drive;
    this.isRightScore = isRightScore;
    this.xSupplier = x;
    this.ySupplier = y;
    this.isDrivingMode = true; // Mark as driving so we don't auto-finish

    rotController = new PIDController(Constants.ROT_HUB_ALIGNMENT_P, 0, 0); 
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
    // rotController.enableContinuousInput(-Math.PI, Math.PI); // Uncomment if using radians and need wrap-around

    // If you need specific pipeline settings, do them here
    // tagID = LimelightHelpers.getFiducialID("limelight"); 
    // ^ moved to execute because ID might change if you switch targets while driving
  }

  @Override
  public void execute() {
    double rotValue = 0;
    double xSpeed = xSupplier.getAsDouble();
    double ySpeed = ySupplier.getAsDouble();
    boolean hasTarget = LimelightHelpers.getTV("limelight");
    
    // Only fetch ID if we have a target, otherwise keep -1 or last known
    if (hasTarget) {
         tagID = LimelightHelpers.getFiducialID("limelight");
    }

    // Logic: If we see the tag, use PID for rotation. 
    // If we don't see the tag, let the driver rotate manually (or stay at 0).
    if (hasTarget) { // && (tagID == targetID) <-- Add this check back if you want to lock to a specific ID
      this.dontSeeTagTimer.reset();

      // Using "limelight" explicitly instead of "" to be safe
      double[] positions = LimelightHelpers.getBotPose_TargetSpace("limelight");
      
      if (positions.length >= 6) {
          SmartDashboard.putNumber("x", positions[2]);
          // Calculate rotation alignment
          rotValue = -rotController.calculate(positions[4]);
      }
    } else {
        // OPTIONAL: If no tag, allow manual rotation? 
        // Currently set to 0 to prevent spinning if tag is lost.
        rotValue = 0; 
    }

    // Apply Control
    m_drive.setControl(
        new SwerveRequest.FieldCentric()
        .withVelocityX(xSpeed)
        .withVelocityY(ySpeed)
        .withRotationalRate(rotValue)
    );

    // Timer logic for "Stationary" mode finishing
    if (!rotController.atSetpoint()) {
       stopTimer.reset();
    }

    SmartDashboard.putNumber("poseValidTimer", stopTimer.get());
  }

  @Override
  public void end(boolean interrupted) {
    // Stop the robot when the command ends
    m_drive.setControl(
        new SwerveRequest.FieldCentric()
        .withVelocityX(0)
        .withVelocityY(0)
        .withRotationalRate(0)
    );
  }

  @Override
  public boolean isFinished() {
    // If we are in "Driving Mode" (button held), NEVER finish automatically.
    // The command ends only when you release the button.
    if (isDrivingMode) {
        return false;
    }

    // If in "Stationary Mode", use the original timer logic
    return this.dontSeeTagTimer.hasElapsed(Constants.DONT_SEE_TAG_WAIT_TIME) ||
           stopTimer.hasElapsed(Constants.POSE_VALIDATION_TIME);
  }

}