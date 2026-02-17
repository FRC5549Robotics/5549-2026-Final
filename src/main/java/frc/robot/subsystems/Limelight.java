// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Vision.LimelightHelpers;
import edu.wpi.first.math.filter.LinearFilter;

public class Limelight extends SubsystemBase {

  public Limelight() {} //create limelight

  private final LinearFilter distanceFilter = LinearFilter.movingAverage(5);

  public double getDistanceToTagMeters() {

    if (!LimelightHelpers.getTV("limelight")) { //if it can't find a tag
        distanceFilter.reset();
        return -1; 
    }

    var pose = LimelightHelpers.getTargetPose3d_CameraSpace("limelight"); //figure out where robot is

    //solve for distance from hub

    double x = pose.getX(); 
    double z = pose.getZ();

    double rawDistance = Math.sqrt(x * x + z * z);

    return distanceFilter.calculate(rawDistance);
  }
}
