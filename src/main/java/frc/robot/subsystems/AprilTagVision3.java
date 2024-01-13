// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.apriltag;

import frc.robot.commands.Vision.AprilTagLocation;
import frc.robot.subsystems.apriltag.*;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.Supplier;

import org.photonvision.PhotonCamera;

public class AprilTagVision3 extends SubsystemBase {

  private double yaww;
  public float target;
  private PhotonCamera camera;

  /** Creates a new AprilTagVision3. */
  public AprilTagVision3() {
    camera = new PhotonCamera("defaultCamera");
  }
  
  public double returnYaw() {
    return yaww;
  }

  @Override
  public void periodic() {
    this.yaww = yaww;

    var result = camera.getLatestResult();

    SmartDashboard.putBoolean("Has Targets", result.hasTargets());
    // SmartDashboard.putBoolean("Hi", false);

    if(result.hasTargets()) {
      var target = result.getBestTarget();
      //turn to target!!
      var yaw = target.getYaw();
      //use trig to find distance (woop woop)
      var pitch = target.getPitch();
      //getBest finds ALL data to target: distance, degree rotation, degree upright, etc.
      var camToTarget = target.getBestCameraToTarget();
      yaww = yaw;

      SmartDashboard.putNumber("Tag Yaw", target.getYaw());

    } else {
      target = 0;
    }

  }
}
