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
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.FieldConstants;
import frc.robot.Robot;

public class PhotonVisionSystem {
    final AprilTagFieldLayout TagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);
    /*
     * IDs 3,4 and 19,20 are on the side of the hub that we can't shoot from, so
     * don't include them
     */
    private final int[] RedHubApriltagIds = new int[] {
            2, /* 3, 4, */ 5, 8, 9, 10, 11
    };
    private final int[] BlueHubApriltagIds = new int[] {
            18, /* 19, 20, */ 21, 24, 25, 26, 27
    };

    private final ApriltagTarget RedHub = new ApriltagTarget(Map.of(
            2, new Translation3d(Inches.of(-23.5), Inches.of(0), Inches.of(33)),
            5, new Translation3d(Inches.of(-23.5), Inches.of(0), Inches.of(33)),
            8, new Translation3d(Inches.of(-23.5), Inches.of(-14), Inches.of(33)),
            9, new Translation3d(Inches.of(-23.5), Inches.of(14), Inches.of(33)),
            10, new Translation3d(Inches.of(-23.5), Inches.of(0), Inches.of(33)),
            11, new Translation3d(Inches.of(-23.5), Inches.of(14), Inches.of(33))));

    private final ApriltagTarget BlueHub = new ApriltagTarget(Map.of(
            18, new Translation3d(Inches.of(-23.5), Inches.of(0), Inches.of(33)),
            21, new Translation3d(Inches.of(-23.5), Inches.of(0), Inches.of(33)),
            24, new Translation3d(Inches.of(-23.5), Inches.of(-14), Inches.of(33)),
            25, new Translation3d(Inches.of(-23.5), Inches.of(14), Inches.of(33)),
            26, new Translation3d(Inches.of(-23.5), Inches.of(0), Inches.of(33)),
            27, new Translation3d(Inches.of(-23.5), Inches.of(14), Inches.of(33))));

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

            /*
             * Each camera gets its own log namespace so they can be reviewed independently
             */
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

    /*
     * Use the current robot heading to keep track of where to target when aiming
     * for the hub
     */
    Supplier<Pose2d> currentRobotPose;
    Consumer<LoggableRobotPose> poseConsumer;

    /* Combined hub target state (best across all cameras) */
    double timeOfLastTrackedHubTarget = 0;
    Pose3d hubTarget = Pose3d.kZero;
    Rotation2d hubHeading = Rotation2d.kZero;

    VisionSystemSim visionSim = new VisionSystemSim("Camera Sim");

    private final NetworkTable cameraTable = NetworkTableInstance.getDefault().getTable("CameraDetails");
    private final StructPublisher<Pose3d> hubTargetPublisher = cameraTable.getStructTopic("HubTarget", Pose3d.struct)
            .publish();
    private final StructPublisher<Rotation2d> hubHeadingPublisher = cameraTable
            .getStructTopic("HubHeading", Rotation2d.struct).publish();
    private final StructArrayPublisher<Pose3d> cameraOffsetPublisher = cameraTable
            .getStructArrayTopic("Offsets", Pose3d.struct)
            .publish();
    private final Pose3d[] cameraOffsetPoses;

    /**
     * Create a multi-camera vision system.
     * 
     * @param poseConsumer     Called with each robot pose estimate from any camera
     * @param currentRobotPose Supplier for the current robot pose (from drivetrain)
     * @param configs          One CameraConfig per physical camera (name +
     *                         transform)
     */
    public PhotonVisionSystem(Consumer<LoggableRobotPose> poseConsumer, Supplier<Pose2d> currentRobotPose,
            CameraConfig... configs) {
        this.poseConsumer = poseConsumer;
        this.currentRobotPose = currentRobotPose;

        cameras = new CameraState[configs.length];
        cameraOffsetPoses = new Pose3d[configs.length];
        for (int i = 0; i < configs.length; i++) {
            cameras[i] = new CameraState(configs[i], TagLayout);
            cameraOffsetPoses[i] = new Pose3d(currentRobotPose.get()).transformBy(configs[i].robotToCamera());
        }

        cameraOffsetPublisher.set(cameraOffsetPoses);

        // ----- Simulation
        if (Robot.isSimulation()) {
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

        double bestAmbiguityThisCycle = Double.MAX_VALUE;
        PhotonTrackedTarget bestHubTargetThisCycle = null;
        Transform3d bestHubCameraTransform = null;

        for (CameraState cam : cameras) {
            if (!Utils.isReplay()) {
                var allResults = cam.camera.getAllUnreadResults();
                var estimates = new ArrayList<LoggableRobotPose>();

                for (var result : allResults) {
                    var estimateOpt = cam.estimator.estimateCoprocMultiTagPose(result);
                    if (estimateOpt.isEmpty()) {
                        estimateOpt = cam.estimator.estimateLowestAmbiguityPose(result);
                    }
                    estimateOpt.ifPresent(val -> {
                        int[] tagIds = val.targetsUsed.stream().mapToInt(t -> t.getFiducialId()).toArray();
                        if (Math.abs(val.estimatedPose.getZ()) > .025) {
                            return;

                        }
                        if (val.estimatedPose.getX() < 0
                                || val.estimatedPose.getX() > FieldConstants.fieldWidth) {
                            return;
                        }
                        if (val.estimatedPose.getY() < 0
                                || val.estimatedPose.getY() > FieldConstants.fieldWidth) {
                            return;
                        }
                        double maxAmbiguity = .4;
                        if (val.targetsUsed.size() == 1 && val.targetsUsed.get(0).poseAmbiguity > maxAmbiguity) {
                            return;
                        }

                        double translationalScoresSum = 0;
                        double angularScoresSum = 0;
                        for (var tag : val.targetsUsed) {
                            var tagDistance = tag.bestCameraToTarget.getTranslation().getNorm();

                            translationalScoresSum += .4 * tagDistance * tagDistance;
                            angularScoresSum += .2 * tagDistance * tagDistance;
                        }

                        // Heavily distrust single tag observations
                        if (val.targetsUsed.size() == 1) {
                            var scale = val.targetsUsed.get(0).poseAmbiguity / maxAmbiguity;
                            translationalScoresSum *= MathUtil.interpolate(10, 50, scale);
                            angularScoresSum *= MathUtil.interpolate(25, 100, scale);
                        }

                        var translationalDivisor = Math.pow(val.targetsUsed.size(), 1.5);
                        var angularDivisor = Math.pow(val.targetsUsed.size(), 3);
                        
                        if (val.estimatedPose.getZ() < 0.6) {
                            estimates.add(
                                    new LoggableRobotPose(val.estimatedPose, val.timestampSeconds, tagIds,
                                            val.strategy, translationalScoresSum / translationalDivisor, angularScoresSum / angularDivisor));
                        }
                    });
                }

                cam.poses = estimates.toArray(new LoggableRobotPose[0]);
            }

            cam.autoReplay.update();

            for (LoggableRobotPose pose : cam.poses) {
                poseConsumer.accept(pose);
            }
        }

        hubTargetPublisher.accept(hubTarget);
        hubHeadingPublisher.accept(hubHeading);
        cameraOffsetPublisher.set(cameraOffsetPoses);
    }

    public void simPeriodic(Pose2d simPose) {
        visionSim.update(simPose);
    }

    /** A Field2d for visualizing our robot and objects on the field. */
    public Field2d getSimDebugField() {
        if (!Robot.isSimulation())
            return null;
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