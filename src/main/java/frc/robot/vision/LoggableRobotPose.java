package frc.robot.vision;

import java.nio.ByteBuffer;
import java.util.Arrays;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.util.struct.Struct;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public class LoggableRobotPose {
    public final Pose3d estimatedPose;
    public final double timestampSeconds;
    public final int[] tagIds;
    public final PoseStrategy strategy;
    public final double translationalScore;
    public final double angularScore;

    public LoggableRobotPose(Pose3d estimatedPose, double timestampSeconds, int[] tagIds, PoseStrategy strategy,
            double translationalScore, double angularScore) {
        this.estimatedPose = estimatedPose;
        this.timestampSeconds = timestampSeconds;
        this.translationalScore = translationalScore;
        this.angularScore = angularScore;
        this.tagIds = new int[8];
        Arrays.fill(this.tagIds, -1);
        if (tagIds != null) {
            System.arraycopy(tagIds, 0, this.tagIds, 0, Math.min(tagIds.length, 8));
        }
        this.strategy = strategy;
    }

    public static final LoggableRobotPoseStruct struct = new LoggableRobotPoseStruct();

    public static class LoggableRobotPoseStruct implements Struct<LoggableRobotPose> {
        @Override
        public Class<LoggableRobotPose> getTypeClass() {
            return LoggableRobotPose.class;
        }

        @Override
        public String getTypeName() {
            return "LoggableRobotPose";
        }

        @Override
        public int getSize() {
            return Pose3d.struct.getSize() + kSizeDouble + 8 * kSizeInt32 + kSizeInt8 + kSizeDouble + kSizeDouble;
        }

        @Override
        public String getSchema() {
            return "Pose3d estimatedPose;double timestampSeconds;int32 tagIds[8];int8 strategy;double translationalScore;double angularScore;";
        }

        @Override
        public Struct<?>[] getNested() {
            return new Struct<?>[] { Pose3d.struct };
        }

        @Override
        public LoggableRobotPose unpack(ByteBuffer bb) {
            Pose3d pose = Pose3d.struct.unpack(bb);
            double time = bb.getDouble();
            int[] tags = new int[8];
            for (int i = 0; i < 8; i++) {
                tags[i] = bb.getInt();
            }
            int strategyOrdinal = bb.get();
            PoseStrategy st = PoseStrategy.LOWEST_AMBIGUITY;
            if (strategyOrdinal >= 0 && strategyOrdinal < PoseStrategy.values().length) {
                st = PoseStrategy.values()[strategyOrdinal];
            }
            double translationalScore = bb.getDouble();
            double angularScore = bb.getDouble();

            return new LoggableRobotPose(pose, time, tags, st, translationalScore, angularScore);
        }

        @Override
        public void pack(ByteBuffer bb, LoggableRobotPose value) {
            Pose3d.struct.pack(bb, value.estimatedPose);
            bb.putDouble(value.timestampSeconds);
            for (int i = 0; i < 8; i++) {
                bb.putInt(value.tagIds[i]);
            }
            bb.put((byte) (value.strategy != null ? value.strategy.ordinal() : 0));
            bb.putDouble(value.translationalScore);
            bb.putDouble(value.angularScore);
        }

        @Override
        public boolean isImmutable() {
            return true;
        }
    }
}