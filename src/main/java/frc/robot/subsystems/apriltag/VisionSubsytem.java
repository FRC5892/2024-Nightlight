// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.apriltag;

import frc.robot.commands.Vision.AprilTagLocation;
import frc.robot.subsystems.apriltag.*;
import frc.robot.Constants.VisionConstants;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public class VisionSubsytem extends SubsystemBase {

  private double yaww;
  public float target;
  private PhotonCamera camera;
  private PhotonPoseEstimator poseEstimator;
  private AprilTagFieldLayout fieldLayout;
  private Pose2d visionPose;
  private Field2d field2d;

  /**
   * Creates a new AprilTagVision3.
   * 
   * @throws IOException
   */
  public VisionSubsytem() throws IOException {
    camera = new PhotonCamera(VisionConstants.CAMERA_NAME);
    fieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
    poseEstimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, camera,
        VisionConstants.ROBOT_TO_CAM);
    field2d = new Field2d();
    SmartDashboard.putData(field2d);
  }

  public double returnYaw() {
    return yaww;
  }
// feed into SwerveDrivePoseEstimator
  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
    poseEstimator.setReferencePose(prevEstimatedRobotPose);
    return poseEstimator.update();
    
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("test", false);
    Optional<EstimatedRobotPose> estimatedPose = getEstimatedGlobalPose(visionPose);
    if (estimatedPose.isPresent()) {
      this.visionPose = getEstimatedGlobalPose(this.visionPose).get().estimatedPose.toPose2d();
    }
    var result = camera.getLatestResult();
    field2d.setRobotPose(this.visionPose);
    


    

    SmartDashboard.putBoolean("Has Targets", result.hasTargets());
    // SmartDashboard.putBoolean("Hi", false);

    if (result.hasTargets()) {
      var target = result.getBestTarget();
      // turn to target!!
      var yaw = target.getYaw();
      // use trig to find distance (woop woop)
      var pitch = target.getPitch();
      // getBest finds ALL data to target: distance, degree rotation, degree upright,
      // etc.
      var camToTarget = target.getBestCameraToTarget();
      yaww = yaw;

      SmartDashboard.putNumber("Tag Yaw", target.getYaw());

    } else {
      target = 0;
    }

  }
}
