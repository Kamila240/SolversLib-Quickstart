package org.firstinspires.ftc.teamcode.subsystems;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public class PedroDriveSubsystem extends SubsystemBase {

    private final Follower follower;
    private final Telemetry telemetry;

    public PedroDriveSubsystem(HardwareMap hardwareMap, Telemetry telemetry, Pose startPose) {
        this(hardwareMap, telemetry, startPose, true);
    }

    public PedroDriveSubsystem(HardwareMap hardwareMap, Telemetry telemetry, Pose startPose, boolean teleopMode) {
        this.telemetry = telemetry;

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        if (teleopMode) {
            follower.startTeleopDrive();
        }

        follower.update();
    }

    public void setTeleOpDrive(double forward, double strafe, double turn, boolean robotCentric) {
        follower.setTeleOpDrive(forward, strafe, turn, robotCentric);
    }

    public void stopDrive(boolean robotCentric) {
        follower.setTeleOpDrive(0, 0, 0, robotCentric);
    }

    public void followPath(Path path) {
        follower.followPath(path);
    }

    public void followPath(PathChain pathChain, boolean holdEnd) {
        follower.followPath(pathChain, holdEnd);
    }

    public boolean isBusy() {
        return follower.isBusy();
    }

    public Pose getPose() {
        return follower.getPose();
    }

    public void setPose(Pose pose) {
        follower.setStartingPose(pose);
    }

    public void setMaxPower(double power) {
        follower.setMaxPower(power);
    }

    public PathBuilder pathBuilder() {
        return follower.pathBuilder();
    }

    public Follower getFollower() {
        return follower;
    }

    public void update() {
        follower.update();
    }

    @Override
    public void periodic() {
        follower.update();

        Pose pose = follower.getPose();
        telemetry.addData("Drive Pose X", pose.getX());
        telemetry.addData("Drive Pose Y", pose.getY());
        telemetry.addData("Drive Heading", Math.toDegrees(pose.getHeading()));
    }
}