package frc.robot.subsystems;

import org.photonvision.PhotonCamera;

// import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PhotonVision extends SubsystemBase {
  PhotonCamera m_camera = new PhotonCamera("HD_Web_Camera");
  public PhotonVision() {
    
  }

  // return if the target is targeted
  public boolean hasTarget() {
    // get photon pipline result
    return m_camera.getLatestResult().hasTargets();
  }

  // return the distance to the target
  public Transform2d getDistance() {
    if (true == hasTarget()) {
      // return targetpose
      try {
        return m_camera.getLatestResult().getTargets().get(0).getCameraToTarget();
      } catch (Exception e) {
        return null;
      }
    } else {
      // return 0
      return null;
    }
  }

  // return the area of the target
  public Double getArea() {
    if (false == hasTarget()) {
      return null;
    } else {
      return m_camera.getLatestResult().getTargets().get(0).getArea();
    }
  }

  // return the pitch of the target
  public Double getPitch () {
    if (hasTarget() == true) {
      return m_camera.getLatestResult().getTargets().get(0).getPitch();
    } else {
      return null;
    }
  }
  public Double getYaw () {
    if (hasTarget() == true) {
      return m_camera.getLatestResult().getTargets().get(0).getYaw();
    } else {
      return null;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}