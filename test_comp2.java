import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
public class test_comp2 {
    public void test(EstimatedRobotPose pose) {
        PoseStrategy strategy = pose.strategy;
        var targets = pose.targetsUsed;
    }
}
