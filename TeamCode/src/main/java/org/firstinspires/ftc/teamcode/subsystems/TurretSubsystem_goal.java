package org.firstinspires.ftc.teamcode.subsystems;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Configs.TurretConfig_goal;

public class TurretSubsystem_goal extends SubsystemBase {

    private final Servo servo1;
    private final Servo servo2;
    private final Telemetry telemetry;

    private double targetAngleDeg = 0.0;
    private double commandedAngleDeg = 0.0;
    private double currentServoPos = TurretConfig_goal.SERVO_CENTER;

    private boolean controlEnabled = false;

    public TurretSubsystem_goal(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        servo1 = hardwareMap.get(Servo.class, "turret1");
        servo2 = hardwareMap.get(Servo.class, "turret2");

        applyDirections();

        center();
        writeServosForAngle(0.0);
    }

    private void applyDirections() {
        servo1.setDirection(
                TurretConfig_goal.REVERSE_SERVO1 ? Servo.Direction.REVERSE : Servo.Direction.FORWARD
        );
        servo2.setDirection(
                TurretConfig_goal.REVERSE_SERVO2 ? Servo.Direction.REVERSE : Servo.Direction.FORWARD
        );
    }

    public void setControlEnabled(boolean enabled) {
        controlEnabled = enabled;

        if (!enabled) {
            targetAngleDeg = 0.0;
            commandedAngleDeg = 0.0;
            writeServosForAngle(0.0);
        }
    }

    public void center() {
        targetAngleDeg = 0.0;
        commandedAngleDeg = 0.0;
        writeServosForAngle(0.0);
    }

    public void setTargetAngleDeg(double angleDeg) {
        if (!controlEnabled) return;
        targetAngleDeg = clipAngle(wrapDeg(angleDeg));
    }

    public void manualControl(double stick) {
        if (!controlEnabled) return;

        if (Math.abs(stick) < TurretConfig_goal.STICK_DEADBAND) {
            return;
        }

        targetAngleDeg += -stick * TurretConfig_goal.MANUAL_STEP_DEG;
        targetAngleDeg = clipAngle(targetAngleDeg);
    }

    public void aimAtFieldPoint(Pose pose, double goalX, double goalY) {
        if (!controlEnabled) return;

        double robotX = pose.getX();
        double robotY = pose.getY();
        double headingDeg = Math.toDegrees(pose.getHeading());

        /*if (robotY >= 72) {
            AutoAimConfig.GOAL_X = 6.849688473520252;
            AutoAimConfig.GOAL_Y = 130.553738317757;
        }
        if (robotY <= 30) {
            AutoAimConfig.GOAL_X = 15.22507788161994;
            AutoAimConfig.GOAL_Y = 140.0311526479751;
        }*/

        double deltaX = goalX - robotX;
        double deltaY = goalY - robotY;

        double absoluteTargetDeg = Math.toDegrees(Math.atan2(deltaY, deltaX));

        /*double offset = 0.0;
        if (headingDeg <= 91){
            offset = TurretConfig_goal.TUR_OFFSETS_LEFT;
        }
        if (headingDeg > 92 && headingDeg < 110){
            offset = TurretConfig_goal.TUR_OFFSET_RIGHT;
        }
        if (headingDeg >= 110){
            offset = TurretConfig_goal.TUR_OFFSET_5TH;
        }
        if (headingDeg >= -178 && headingDeg < -90){
            offset = TurretConfig_goal.TUR_OFFSET_3RD;
            //headingDeg = headingDeg;
        }
        if (headingDeg >= -90 && headingDeg < 0){
            offset = TurretConfig_goal.TUR_OFFSET_4TH;
            //headingDeg = headingDeg + 180;
        }*/

        double relativeTurretAngleDeg = wrapDeg(absoluteTargetDeg - headingDeg) + TurretConfig_goal.OFFSET;
        setTargetAngleDeg(relativeTurretAngleDeg);

        telemetry.addData("Robot X", robotX);
        telemetry.addData("Robot Y", robotY);
        telemetry.addData("Heading Deg", headingDeg);
        telemetry.addData("Goal X", goalX);
        telemetry.addData("Goal Y", goalY);
        telemetry.addData("Absolute Target Deg", absoluteTargetDeg);
        telemetry.addData("Relative Turret Deg", relativeTurretAngleDeg);
    }

    public double getTargetAngleDeg() {
        return targetAngleDeg;
    }

    public double getCommandedAngleDeg() {
        return commandedAngleDeg;
    }

    public double getCurrentServoPos() {
        return currentServoPos;
    }

    private void updateServoCommand() {
        double errorDeg = targetAngleDeg - commandedAngleDeg;
        double maxStep = Math.max(TurretConfig_goal.MAX_STEP_DEG_PER_LOOP, 0.001);

        if (Math.abs(errorDeg) <= maxStep) {
            commandedAngleDeg = targetAngleDeg;
        } else {
            commandedAngleDeg += Math.signum(errorDeg) * maxStep;
        }

        commandedAngleDeg = clipAngle(commandedAngleDeg);
        writeServosForAngle(commandedAngleDeg);
    }

    private void writeServosForAngle(double angleDeg) {
        //currentServoPos = angleToServoPosition(angleDeg);
        double servoAngleDeg = angleDeg / TurretConfig_goal.GEAR_RATIO;
        currentServoPos = angleToServoPosition(servoAngleDeg);
        servo1.setPosition(currentServoPos);
        servo2.setPosition(currentServoPos);
    }

    private double angleToServoPosition(double servoAngleDeg) {
        double halfRange = TurretConfig_goal.SERVO_RANGE_DEG / 2.0;
        double clippedAngle = Range.clip(servoAngleDeg, -halfRange, halfRange);

        double servoPos = TurretConfig_goal.SERVO_CENTER + (clippedAngle / TurretConfig_goal.SERVO_RANGE_DEG);

        return Range.clip(servoPos, TurretConfig_goal.SERVO_MIN, TurretConfig_goal.SERVO_MAX);
    }


    private double clipAngle(double turretAngleDeg) {
        double halfTurretRange = (TurretConfig_goal.SERVO_RANGE_DEG * TurretConfig_goal.GEAR_RATIO) / 2.0;
        return Range.clip(turretAngleDeg, -halfTurretRange, halfTurretRange);
    }

    private double wrapDeg(double angleDeg) {
        while (angleDeg > 180.0) angleDeg -= 360.0;
        while (angleDeg < -180.0) angleDeg += 360.0;
        return angleDeg;
    }

    @Override
    public void periodic() {
        applyDirections();
        updateServoCommand();

        telemetry.addData("Turret Enabled", controlEnabled);
        telemetry.addData("Turret Target Deg", targetAngleDeg);
        telemetry.addData("Turret Commanded Deg", commandedAngleDeg);
        telemetry.addData("Turret Servo Pos", currentServoPos);
    }
}