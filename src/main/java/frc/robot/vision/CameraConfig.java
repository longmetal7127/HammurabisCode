package frc.robot.vision;

import edu.wpi.first.math.geometry.Transform3d;

/**
 * Configuration for a single PhotonVision camera.
 * @param name The NetworkTables name of the camera (must match what's configured in PhotonVision)
 * @param robotToCamera The 3D transform from the robot center to the camera's mounting position/orientation
 */
public record CameraConfig(String name, Transform3d robotToCamera) {}
