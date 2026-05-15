package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
@TeleOp(name = "Turret Servo Calibration", group = "Calibration")
public class TurretServoCalibration extends LinearOpMode {

    private Servo servo1;
    private Servo servo2;

    private double servoPos = 0.50;

    private static final double SMALL_STEP = 0.001;
    private static final double BIG_STEP = 0.01;

    private static final boolean REVERSE_SERVO1 = false;
    private static final boolean REVERSE_SERVO2 = false;

    @Override
    public void runOpMode() {
        servo1 = hardwareMap.get(Servo.class, "turret1");
        servo2 = hardwareMap.get(Servo.class, "turret2");
        //servo3 = hardwareMap.get(Servo.class, "servo1");

        if (REVERSE_SERVO1) {
            servo1.setDirection(Servo.Direction.REVERSE);
        }

        if (REVERSE_SERVO2) {
            servo2.setDirection(Servo.Direction.REVERSE);
        }

        servo1.setPosition(servoPos);
        servo2.setPosition(servoPos);

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.dpad_right) {
                servoPos += SMALL_STEP;
            }
            if (gamepad1.dpad_left) {
                servoPos -= SMALL_STEP;
            }

            if (gamepad1.right_bumper) {
                servoPos += BIG_STEP;
            }
            if (gamepad1.left_bumper) {
                servoPos -= BIG_STEP;
            }

            if (gamepad1.a) {
                servoPos = 0.0;
            }
            if (gamepad1.b) {
                servoPos = 0.5;
            }
            if (gamepad1.y) {
                servoPos = 1.0;
            }

            servoPos = Range.clip(servoPos, 0.0, 1.0);

            servo1.setPosition(servoPos);
            servo2.setPosition(servoPos);

            telemetry.addLine("=== TURRET SERVO CALIBRATION ===");
            telemetry.addData("Servo Pos", "%.4f", servoPos);
            telemetry.addData("Servo1 Reverse", REVERSE_SERVO1);
            telemetry.addData("Servo2 Reverse", REVERSE_SERVO2);
            telemetry.addLine();
            telemetry.addData("DPAD LEFT/RIGHT", "small step");
            telemetry.addData("LB/RB", "big step");
            telemetry.addData("A", "set 0.0");
            telemetry.addData("B", "set 0.5");
            telemetry.addData("Y", "set 1.0");
            telemetry.addLine();
            telemetry.addLine("Find:");
            telemetry.addLine("1) left safe limit");
            telemetry.addLine("2) center");
            telemetry.addLine("3) right safe limit");

            telemetry.update();
        }
    }
}