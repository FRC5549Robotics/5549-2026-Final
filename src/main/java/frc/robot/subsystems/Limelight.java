// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;
import java.util.function.DoubleSupplier;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Vision.LimelightHelpers;
import frc.robot.Vision.LimelightHelpers.RawFiducial;


public class Limelight extends SubsystemBase {
  /** Creates a new Limelight. */
  PhotonCamera camera;
  CommandSwerveDrivetrain m_drivetrain;
  CommandXboxController xbox_controller;
  PIDController rotationController = new PIDController(0.08, 0, 0.001);
  PIDController translationController = new PIDController(2, 0, 0.002);
  NetworkTable limelightTable;

  public Limelight(CommandSwerveDrivetrain drivetrain, CommandXboxController xcontroller) {
    m_drivetrain = drivetrain;
    xbox_controller = xcontroller;
    limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
  }

  private int lastTag = -1;

  public Command alignToTargetCommand() {

    return this.run(() -> {
      double rotSpeed = 0.0;
      
      // Check if we see a target
      if (LimelightHelpers.getTV("limelight")) {

        int tagID = (int) LimelightHelpers.getFiducialID("limelight");

        int desiredPipeline = 0;

        //if (tagID != lastTag) {

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
        }

        int currentPipeline = (int) LimelightHelpers.getCurrentPipelineIndex("limelight");

        if (currentPipeline != desiredPipeline) {
          LimelightHelpers.setPipelineIndex("limelight", desiredPipeline);
        }

        //DriverStation.reportWarning("Tag=" + tagID + " Pipeline=" + desiredPipeline, false);
        

        // tx is the horizontal offset in degrees. 0 = centered.
        double tx = LimelightHelpers.getTX("limelight");
        
        // Calculate PID output to get tx to 0
        rotSpeed = rotationController.calculate(tx, 0); 
      }

      // Use the method built into your CommandSwerveDrivetrain
      // This is robot-centric, which is fine for spinning in place.
      m_drivetrain.driveRobotRelative(new ChassisSpeeds(0, 0, rotSpeed));
    });
  }

  // ---------------------------------------------------------
  // OPTION 2: Drive manually (Field Centric) while auto-aiming
  // ---------------------------------------------------------
  // We need a FieldCentric request object for this command
  private final SwerveRequest.FieldCentric driveFieldCentric = new SwerveRequest.FieldCentric();

  public Command alignWhileDrivingCommand(DoubleSupplier xSpeed, DoubleSupplier ySpeed) {
  // We use Commands.run so we can add the 'm_drivetrain' requirement.
  // This ensures the default drive command stops while this is running.
    return Commands.run(() -> {
      double rotSpeed = 0.0;

    // Only calculate rotation if we see a target
      if (LimelightHelpers.getTV("limelight")) {
        double tx = LimelightHelpers.getTX("limelight");
      // Calculate output to get tx to 0
        rotSpeed = rotationController.calculate(tx, 0); 
      }

    // driveFieldCentric is a raw request, so we apply the speeds directly.
      m_drivetrain.setControl(driveFieldCentric
        .withVelocityX(xSpeed.getAsDouble())
        .withVelocityY(ySpeed.getAsDouble())
        .withRotationalRate(rotSpeed) // The auto-aim rotation
      );
    }, m_drivetrain, this); // REQUIREMENTS: Drivetrain + Limelight
  }


  public double[] turnToTarget(Boolean isRightScore) {
    //check for target
    boolean hasTarget = LimelightHelpers.getTV("limelight");
    if (hasTarget) {
      double [] targetSpaceArray = LimelightHelpers.getBotPose_TargetSpace("limelight");
      Pose3d targetSpacePose = LimelightHelpers.getBotPose3d_TargetSpace("limelight");
      double currentRotation = targetSpaceArray[4];
      double rotationError = currentRotation - 4.6;

      if (xbox_controller.getHID().getLeftBumperButton()){
        double xSpeed = translationController.calculate(0, targetSpacePose.getZ() + 0.45);
        double ySpeed = translationController.calculate(0,-targetSpacePose.getX() - 0.24);
        double rotSpeed = rotationController.calculate(rotationError, 0);

        return new double []{xSpeed, ySpeed, rotSpeed};
      }
      else if (xbox_controller.getHID().getRightBumperButton()){
        double xSpeed = translationController.calculate(0, targetSpacePose.getZ() + 0.5);
        double ySpeed = translationController.calculate(0,-targetSpacePose.getX() + 0.09);
        double rotSpeed = rotationController.calculate(rotationError, 0);

        return new double [] {xSpeed, ySpeed, rotSpeed};
      }
    }
    return null;
    }
    // thetaController.setSetpoint(0);
    // thetaController.setTolerance(0);

    // xController.setSetpoint(0);
    // xController.setTolerance(0);

    // yController.setSetpoint(isRightScore ? Constants.Y_SETPOINT_RIGHT_REEF_ALIGNMENT : Constants.Y_SETPOINT_LEFT_REEF_ALIGNMENT);
    // yController.setTolerance(Constants.Y_TOLERANCE_REEF_ALIGNMENT);

    // if (LimelightHelpers.getTV("limelight")) {
    //   double[] s = LimelightHelpers.getBotPose_TargetSpace("limelight");
    //   // Pose3d bot = LimelightHelpers.getBotPose3d_wpiBlue("limelight");
    //   Pose3d ttr = LimelightHelpers.getBotPose3d_TargetSpace("limelight");
    //   double[] speeds = {xController.calculate(ttr.getZ()), yController.calculate(ttr.getX()), thetaController.calculate(s[4])};
    //   return speeds;
    // }


  //   double[] s = LimelightHelpers.getBotPose_TargetSpace("limelight");
  //     // Pose3d bot = LimelightHelpers.getBotPose3d_wpiBlue("limelight");
  //   Pose3d ttr = LimelightHelpers.getBotPose3d_TargetSpace("limelight");
  //   double angle = s[4] - 4.6;
    
  //   if(LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight").tagCount > 0){
  //     if (xbox_controller.getHID().getLeftBumperButton()){
  //       double[] speeds = {controller2.calculate(0, ttr.getZ()+0.45), controller2.calculate(0, -ttr.getX()-.24), controller.calculate(angle, 0)};
  //       return speeds;
  //     }
  //     else if (xbox_controller.getHID().getRightBumperButton()) {
  //       double[] speeds = {controller2.calculate(0, ttr.getZ()+0.5), controller2.calculate(0, -ttr.getX()+.09), controller.calculate(angle, 0)};
  //       return speeds;
  //    }
  //   }
  //   return null;
  // }
  
  public double getDistanceToTagMeters() {

    if (!LimelightHelpers.getTV("limelight")) {
        return -1;
    }

    var pose = LimelightHelpers.getTargetPose3d_CameraSpace("limelight");

    double x = pose.getX();
    double z = pose.getZ();

    return Math.sqrt(x * x + z * z);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // double[] s = LimelightHelpers.getBotPose_TargetSpace("limelight");
    // System.out.println(s);
    // var s = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose_targetspace").getDoubleArray(new double[6]);
    // System.out.println(s);

    // for (int i = 0; i<s.length; i++) {
    //   System.out.println(s[i]);
    // }
    // System.out.println(s[4]);

    
  }}
