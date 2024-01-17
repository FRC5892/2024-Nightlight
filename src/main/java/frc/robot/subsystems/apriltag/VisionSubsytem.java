// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.apriltag;

import frc.robot.commands.Vision.AddVisionPose;
import frc.robot.subsystems.apriltag.*;
import frc.robot.Robot;
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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.io.IOException;
import java.io.UncheckedIOException;
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

  public float target;
  private PhotonCamera camera;
  private PhotonPoseEstimator poseEstimator;
  private AprilTagFieldLayout fieldLayout;
  private double poseTimestamp;
  private Pose2d visionPose = new Pose2d(0.0, 0.0, new Rotation2d(0.0));
  private Field2d field2d;
  private Pose2d referencePose = new Pose2d(0.0, 0.0, new Rotation2d(0.0));

  /**
   * Creates a new AprilTagVision3.
   * 
   * @throws IOException
   */
  public VisionSubsytem() {

    camera = new PhotonCamera(VisionConstants.CAMERA_NAME);
    try {
      fieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
    } catch (IOException e) {
      throw new UncheckedIOException(e);
    }
    poseEstimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, camera,
        VisionConstants.ROBOT_TO_CAM);
    field2d = new Field2d();
    SmartDashboard.putData("Vision estimated Pose",field2d);

    poseTimestamp = Timer.getFPGATimestamp();
  }
  
  // feed into SwerveDrivePoseEstimator
  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
    poseEstimator.setReferencePose(prevEstimatedRobotPose);
    return poseEstimator.update();
    
  }
  public Pose2d getVisionPose() {
    return visionPose;
  } 
  public Double getVisionTimestamp () {
    return poseTimestamp;
  }
  
  public Pose2d getReferencePose() {
    return referencePose;
  }
  public void setReferencePose(Pose2d referencePose) {
    this.referencePose = referencePose;
  } 
  @Override
  public void periodic() {
    Optional<EstimatedRobotPose> estimatedPose = getEstimatedGlobalPose(referencePose);
    if (estimatedPose.isPresent()) {
      this.visionPose = estimatedPose.get().estimatedPose.toPose2d();
      this.poseTimestamp = estimatedPose.get().timestampSeconds;
    }
    var result = camera.getLatestResult();
    field2d.setRobotPose(this.visionPose);
    
    


    
    SmartDashboard.putNumber("Vision estimated Angle",getVisionPose().getRotation().getDegrees());
    SmartDashboard.putBoolean("Has Targets", result.hasTargets());
    // SmartDashboard.putBoolean("Hi", false);

    if (result.hasTargets()) {
      // field2d.getObject("apriltag").setPoses(result.getTargets().);

      var target = result.getBestTarget();
      // turn to target!!
      var yaw = target.getYaw();
      // use trig to find distance (woop woop)
      var pitch = target.getPitch();
      // getBest finds ALL data to target: distance, degree rotation, degree upright,
      // etc.
      var camToTarget = target.getBestCameraToTarget();

      SmartDashboard.putNumber("Tag Yaw", target.getYaw());

    } else {
      target = 0;
    }

  }
}
