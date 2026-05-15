package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@Config
@TeleOp(name = "Servo Dashboard Test")
public class ServoDashboardTest extends LinearOpMode {

    public static double SERVO_POS = 0.5;
    public static double SERVO_MIN = 0.0;
    public static double SERVO_MAX = 1.0;
    public static boolean REVERSE_SERVO = false;

    private Servo testServo;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(
                telemetry,
                FtcDashboard.getInstance().getTelemetry()
        );

        testServo = hardwareMap.get(Servo.class, "stopper");

        waitForStart();

        while (opModeIsActive()) {
            testServo.setDirection(
                    REVERSE_SERVO ? Servo.Direction.REVERSE : Servo.Direction.FORWARD
            );

            double clippedPos = Range.clip(SERVO_POS, SERVO_MIN, SERVO_MAX);
            testServo.setPosition(clippedPos);

            telemetry.addData("Servo Pos", clippedPos);
            telemetry.addData("Servo Min", SERVO_MIN);
            telemetry.addData("Servo Max", SERVO_MAX);
            telemetry.addData("Servo Direction", REVERSE_SERVO ? "REVERSE" : "FORWARD");
            telemetry.update();
        }
    }
}