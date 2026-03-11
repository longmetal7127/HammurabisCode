package frc.robot.vision;

import static edu.wpi.first.units.Units.Inches;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Map;
import java.util.function.Consumer;
import java.util.function.Supplier;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import com.ctre.phoenix6.HootAutoReplay;
import com.ctre.phoenix6.Utils;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.Robot;

public class PhotonVisionSystem {
    final AprilTagFieldLayout TagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    /* IDs 3,4 and 19,20 are on the side of the hub that we can't shoot from, so don't include them */
    private final int[] RedHubApriltagIds = new int[]{
        2, /* 3, 4, */ 5, 8, 9, 10, 11
    };
    private final int[] BlueHubApriltagIds = new int[]{
        18, /* 19, 20, */ 21, 24, 25, 26, 27
    };

    private final ApriltagTarget RedHub = new ApriltagTarget(Map.of(
        2, new Translation3d(Inches.of(-23.5), Inches.of(0), Inches.of(33)),
        5, new Translation3d(Inches.of(-23.5), Inches.of(0), Inches.of(33)),
        8, new Translation3d(Inches.of(-23.5), Inches.of(-14), Inches.of(33)),
        9, new Translation3d(Inches.of(-23.5), Inches.of(14), Inches.of(33)),
        10, new Translation3d(Inches.of(-23.5), Inches.of(0), Inches.of(33)),
        11, new Translation3d(Inches.of(-23.5), Inches.of(14), Inches.of(33))
    ));

    private final ApriltagTarget BlueHub = new ApriltagTarget(Map.of(
        18, new Translation3d(Inches.of(-23.5), Inches.of(0), Inches.of(33)),
        21, new Translation3d(Inches.of(-23.5), Inches.of(0), Inches.of(33)),
        24, new Translation3d(Inches.of(-23.5), Inches.of(-14), Inches.of(33)),
        25, new Translation3d(Inches.of(-23.5), Inches.of(14), Inches.of(33)),
        26, new Translation3d(Inches.of(-23.5), Inches.of(0), Inches.of(33)),
        27, new Translation3d(Inches.of(-23.5), Inches.of(14), Inches.of(33))
    ));

    /**
     * Per-camera runtime state. Each camera has its own PhotonCamera, estimator,
     * pose array, and HootAutoReplay entries so they log independently.
     */
    private static class CameraState {
        final String name;
        final PhotonCamera camera;
        final Transform3d robotToCamera;
        final PhotonPoseEstimator estimator;
        PhotonCameraSim cameraSim;

        /* Per-camera logged data */
        LoggableRobotPose[] poses = new LoggableRobotPose[0];
        double timeOfLastTrackedHubTarget = 0;
        PhotonTrackedTarget lastTrackedHubTarget = new PhotonTrackedTarget(
            0, 0, 0, 0, -1, -1, 0, Transform3d.kZero, Transform3d.kZero, 0,
            new ArrayList<TargetCorner>(), new ArrayList<TargetCorner>());
        Pose3d hubTarget = Pose3d.kZero;
        Rotation2d hubHeading = Rotation2d.kZero;

        final HootAutoReplay autoReplay;

        CameraState(CameraConfig config, AprilTagFieldLayout tagLayout) {
            this.name = config.name();
            this.camera = new PhotonCamera(config.name());
            this.robotToCamera = config.robotToCamera();
            this.estimator = new PhotonPoseEstimator(tagLayout, config.robotToCamera());

            /* Each camera gets its own log namespace so they can be reviewed independently */
            this.autoReplay = new HootAutoReplay()
                .withStructArray(name + "/PoseEstimations", LoggableRobotPose.struct,
                    () -> poses, val -> poses = val.value)
                .withStruct(name + "/HubTarget", Pose3d.struct,
                    () -> hubTarget, val -> hubTarget = val.value)
                .withStruct(name + "/HubHeading", Rotation2d.struct,
                    () -> hubHeading, val -> hubHeading = val.value)
                .withProtobuf(name + "/LastTrackedHubTarget", PhotonTrackedTarget.proto,
                    () -> lastTrackedHubTarget, val -> lastTrackedHubTarget = val.value)
                .withDouble(name + "/LastTrackedHubTargetTime",
                    () -> timeOfLastTrackedHubTarget, val -> timeOfLastTrackedHubTarget = val.value);
        }
    }

    private final CameraState[] cameras;

    /* Use the current robot heading to keep track of where to target when aiming for the hub */
    Supplier<Pose2d> currentRobotPose;
    Consumer<LoggableRobotPose> poseConsumer;

    /* Combined hub target state (best across all cameras) */
    double timeOfLastTrackedHubTarget = 0;
    Pose3d hubTarget = Pose3d.kZero;
    Rotation2d hubHeading = Rotation2d.kZero;

    VisionSystemSim visionSim = new VisionSystemSim("Camera Sim");

    private final NetworkTable cameraTable = NetworkTableInstance.getDefault().getTable("CameraDetails");
    private final StructPublisher<Pose3d> hubTargetPublisher = cameraTable.getStructTopic("HubTarget", Pose3d.struct).publish();
    private final StructPublisher<Rotation2d> hubHeadingPublisher = cameraTable.getStructTopic("HubHeading", Rotation2d.struct).publish();

    /**
     * Create a multi-camera vision system.
     * @param poseConsumer Called with each robot pose estimate from any camera
     * @param currentRobotPose Supplier for the current robot pose (from drivetrain)
     * @param configs One CameraConfig per physical camera (name + transform)
     */
    public PhotonVisionSystem(Consumer<LoggableRobotPose> poseConsumer, Supplier<Pose2d> currentRobotPose,
                              CameraConfig... configs) {
        this.poseConsumer = poseConsumer;
        this.currentRobotPose = currentRobotPose;

        cameras = new CameraState[configs.length];
        for (int i = 0; i < configs.length; i++) {
            cameras[i] = new CameraState(configs[i], TagLayout);
        }

        // ----- Simulation
        if (Robot.isSimulation() && false) {
            visionSim = new VisionSystemSim("main");
            visionSim.addAprilTags(TagLayout);
            for (CameraState cam : cameras) {
                cam.cameraSim = new PhotonCameraSim(cam.camera);
                visionSim.addCamera(cam.cameraSim, cam.robotToCamera);
                cam.cameraSim.enableDrawWireframe(true);
            }
        }
    }

    public void periodic() {
        Alliance currentAlliance = DriverStation.getAlliance().orElse(Alliance.Red);

        int[] hubTargetIds;
        if (currentAlliance == Alliance.Red) {
            hubTargetIds = RedHubApriltagIds;
        } else {
            hubTargetIds = BlueHubApriltagIds;
        }

        double bestAmbiguityThisCycle = Double.MAX_VALUE;
        PhotonTrackedTarget bestHubTargetThisCycle = null;
        Transform3d bestHubCameraTransform = null;

        for (CameraState cam : cameras) {
            if (!Utils.isReplay()) {
                var allResults = cam.camera.getAllUnreadResults();
                var estimates = new ArrayList<LoggableRobotPose>();

                for (var result : allResults) {
                    var allTargets = result.getTargets();

                    PhotonTrackedTarget bestTargetInResult = null;
                    for (PhotonTrackedTarget target : allTargets) {
                        if (Arrays.stream(hubTargetIds).anyMatch(x -> x == target.fiducialId)) {
                            if (bestTargetInResult == null) {
                                bestTargetInResult = target;
                            } else if (target.poseAmbiguity < bestTargetInResult.poseAmbiguity
                                       && target.poseAmbiguity > 0) {
                                bestTargetInResult = target;
                            }
                        }
                    }

                    if (bestTargetInResult != null) {
                        cam.lastTrackedHubTarget = bestTargetInResult;
                        cam.timeOfLastTrackedHubTarget = Utils.getCurrentTimeSeconds();
                        Transform3d tagRelativeToRobot = bestTargetInResult.bestCameraToTarget;
                        var transformToHub = currentAlliance == Alliance.Red
                            ? RedHub.getHubPose(bestTargetInResult.fiducialId)
                            : BlueHub.getHubPose(bestTargetInResult.fiducialId);
                        var robotPose = currentRobotPose.get();
                        cam.hubTarget = new Pose3d(robotPose)
                            .transformBy(cam.robotToCamera)
                            .transformBy(tagRelativeToRobot)
                            .transformBy(transformToHub);
                        var hubRelativeToRobot = cam.hubTarget.relativeTo(new Pose3d(robotPose));
                        cam.hubHeading = robotPose.getRotation().plus(
                            hubRelativeToRobot.getTranslation().toTranslation2d().getAngle()
                                .plus(currentAlliance == Alliance.Blue ? Rotation2d.k180deg : Rotation2d.kZero));

                        /* Check if this is the best hub target across all cameras */
                        if (bestTargetInResult.poseAmbiguity < bestAmbiguityThisCycle
                            || bestHubTargetThisCycle == null) {
                            bestAmbiguityThisCycle = bestTargetInResult.poseAmbiguity;
                            bestHubTargetThisCycle = bestTargetInResult;
                            bestHubCameraTransform = cam.robotToCamera;
                        }
                    }

                    var estimate = cam.estimator.estimateCoprocMultiTagPose(result);
                    if (estimate.isEmpty()) {
                        estimate = cam.estimator.estimateLowestAmbiguityPose(result);
                    }
                    estimate.ifPresent(val -> estimates.add(
                        new LoggableRobotPose(val.estimatedPose, val.timestampSeconds)));
                }

                cam.poses = estimates.toArray(new LoggableRobotPose[0]);
            }

            cam.autoReplay.update();

            for (LoggableRobotPose pose : cam.poses) {
                poseConsumer.accept(pose);
            }
        }

        if (bestHubTargetThisCycle != null) {
            timeOfLastTrackedHubTarget = Utils.getCurrentTimeSeconds();
            Transform3d tagRelativeToRobot = bestHubTargetThisCycle.bestCameraToTarget;
            var transformToHub = currentAlliance == Alliance.Red
                ? RedHub.getHubPose(bestHubTargetThisCycle.fiducialId)
                : BlueHub.getHubPose(bestHubTargetThisCycle.fiducialId);
            var robotPose = currentRobotPose.get();
            hubTarget = new Pose3d(robotPose)
                .transformBy(bestHubCameraTransform)
                .transformBy(tagRelativeToRobot)
                .transformBy(transformToHub);
            var hubRelativeToRobot = hubTarget.relativeTo(new Pose3d(robotPose));
            hubHeading = robotPose.getRotation().plus(
                hubRelativeToRobot.getTranslation().toTranslation2d().getAngle()
                    .plus(currentAlliance == Alliance.Blue ? Rotation2d.k180deg : Rotation2d.kZero));
        }

        hubTargetPublisher.accept(hubTarget);
        hubHeadingPublisher.accept(hubHeading);
    }

    public void simPeriodic(Pose2d simPose) {
        visionSim.update(simPose);
    }

    /** A Field2d for visualizing our robot and objects on the field. */
    public Field2d getSimDebugField() {
        if (!Robot.isSimulation()) return null;
        return visionSim.getDebugField();
    }

    public boolean isHubTargetValid() {
        return Utils.getCurrentTimeSeconds() - timeOfLastTrackedHubTarget < 0.2;
    }

    public Pose3d getHubPoseRelativeToRobot() {
        return hubTarget;
    }

    public Rotation2d getHeadingToHubFieldRelative() {
        return hubHeading;
    }
}