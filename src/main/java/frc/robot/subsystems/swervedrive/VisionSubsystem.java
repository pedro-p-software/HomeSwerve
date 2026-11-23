// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervedrive;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {

  //Nome temp  
  private PhotonCamera camera = new PhotonCamera("Arducam sla");
  private PhotonPoseEstimator poseEstimator;
  
  public VisionSubsystem() {
    
    AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);
    
    //placeholder values since we got no camera
    Transform3d robotToCam = new Transform3d(
      new Translation3d(Units.inchesToMeters(10.0), Units.inchesToMeters(0), Units.inchesToMeters(12.0)),
      new Rotation3d(0.0, Units.degreesToRadians(-15.0), 0.0)
    );
  
    poseEstimator = new PhotonPoseEstimator(layout, robotToCam);
  }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
        Optional<EstimatedRobotPose> visionEst = Optional.empty();

        for (var result : camera.getAllUnreadResults()) {
        
            // Try MultiTag on the Orange Pi first; fall back to single-tag lowest ambiguity
            visionEst = poseEstimator.estimateCoprocMultiTagPose(result);
            if (visionEst.isEmpty()) {
                visionEst = poseEstimator.estimateLowestAmbiguityPose(result);
            }
        }
        return visionEst;
    
    }
  
  @Override
  public void periodic() {

  }
}
