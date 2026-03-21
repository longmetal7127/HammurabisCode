import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
public class test_comp_3 {
    public void test(EstimatedRobotPose pose) {
        PoseStrategy strategy = pose.strategy;
        var targets = pose.targetsUsed;
        int t = targets.get(0).getFiducialId();
    }
}
