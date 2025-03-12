// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.PhotonCamera;

import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
  /** Creates a new Vision. */
  PhotonCamera camera;
  double x_axis, y_axis, z_angle;
  Integer id;
  
  // private final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(6.5);
  // private final double HEIGHT_APRILTAGS_PIPEBASE= Units.inchesToMeters(12); //8.75
  // private final double HEIGHT_APRILTAGS_CORALSTATION = Units.inchesToMeters(55.25);
  // private final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(0);
  public Vision() {
    camera = new PhotonCamera("photonvision");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    displayOnDasdhboard();
    updateDistanceVision();
    
  }


  private void updateDistanceVision() {
    System.out.println("Displaying on Dashboard...");
    List<PhotonPipelineResult> results = camera.getAllUnreadResults();
    System.out.println("Number of results: " + results.size());
    for (PhotonPipelineResult result : results) {
        if (result.hasTargets()) {
            System.out.println("Target detected!");
            PhotonTrackedTarget bestTarget = result.getBestTarget();
            x_axis = bestTarget.getAlternateCameraToTarget().getX();
            y_axis = bestTarget.getAlternateCameraToTarget().getY();
            z_angle = bestTarget.getYaw();
            id = bestTarget.getFiducialId();
            // z_axis = bestTarget.getAlternateCameraToTarget().getZ();
            // distance_c = Math.sqrt(Math.pow(x_axis, 2) + Math.pow(y_axis, 2) + Math.pow(z_axis,2));

            // double yaw = bestTarget.getYaw();
            // double pitch = bestTarget.getPitch();
            // double pitch_to_radians = Units.degreesToRadians(pitch);

            // Calculate the distance to the target
            // double range = PhotonUtils.calculateDistanceToTargetMeters(
            //     CAMERA_HEIGHT_METERS,
            //     HEIGHT_APRILTAGS_PIPEBASE,
            //     0,
            //     pitch_to_radians
            // );
            // double radianes = (Units.degreesToRadians(0))+pitch_to_radians;
            // double distance = (HEIGHT_APRILTAGS_PIPEBASE-CAMERA_HEIGHT_METERS)/Math.tan(radianes);

            // System.out.println("Yaw: "+ yaw);
            // System.out.println("Pitch: "+pitch);
            // System.out.println("Distance: "+distance);
        } else {
            System.out.println("No targets detected.");
        }
    }
}
  public void displayOnDasdhboard(){
    System.out.println("X axis:"+x_axis);
    System.out.println("Y axis:"+y_axis);
    System.out.println("Z axis:"+z_angle);
    System.out.println("ID AprilTag"+id);

  }

  public double getXAxis(){return x_axis;};
  public double getYAxis(){return y_axis;};
  public double getZAngle(){return z_angle;};
  public int getIDApriltag(){return id;};
}
