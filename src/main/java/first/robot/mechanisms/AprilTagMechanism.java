package first.robot.mechanisms;

import org.wpilib.command3.Mechanism;

public class AprilTagMechanism implements Mechanism {

    // 希望機器人停在 Tag 前方幾公尺
    private static final double STOP_DISTANCE_METERS = 1.0;

    public AprilTagMechanism() {}


    // =========================================================
    // 1. 根據 AprilTag 的位置，算出機器人應該停的位置
    // =========================================================

    public AlignmentTarget calculateTarget(
        double tagX,
        double tagY,
        double tagYawRadians
    ) {

        // Tag 前方 1 公尺的位置
        double targetX =
            tagX
            + STOP_DISTANCE_METERS
            * Math.cos(tagYawRadians);

        double targetY =
            tagY
            + STOP_DISTANCE_METERS
            * Math.sin(tagYawRadians);

        // 機器人停好後要面向 Tag
        double targetYaw =
            Math.atan2(
                tagY - targetY,
                tagX - targetX
            );

        return new AlignmentTarget(
            targetX,
            targetY,
            targetYaw
        );
    }


    // =========================================================
    // 2. 計算目前機器人跟目標還差多少
    // =========================================================

    public AlignmentError calculateError(
        double robotX,
        double robotY,
        double robotYawRadians,
        AlignmentTarget target
    ) {

        // X 還差多少
        double errorX =
            target.x() - robotX;

        // Y 還差多少
        double errorY =
            target.y() - robotY;

        // 距離目標還有多遠
        double positionError =
            Math.hypot(
                errorX,
                errorY
            );

        // 角度還差多少
        double signedYawError =
            wrapAngleRadians(
                target.yawRadians()
                - robotYawRadians
            );

        // 絕對角度誤差
        double yawError =
            Math.abs(signedYawError);

        return new AlignmentError(
            errorX,
            errorY,
            positionError,
            signedYawError,
            yawError
        );
    }


    // =========================================================
    // 3. 把角度限制在 -180° ~ +180°
    // =========================================================

    private static double wrapAngleRadians(
        double angle
    ) {

        return Math.atan2(
            Math.sin(angle),
            Math.cos(angle)
        );
    }


    // =========================================================
    // 儲存「機器人要去哪裡」
    // =========================================================

    public record AlignmentTarget(
        double x,
        double y,
        double yawRadians
    ) {}


    // =========================================================
    // 儲存「現在還差多少」
    // =========================================================

    public record AlignmentError(
        double errorX,
        double errorY,
        double positionError,
        double signedYawError,
        double yawError
    ) {}
}