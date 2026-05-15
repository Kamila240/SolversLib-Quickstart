package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp(name = "Axon_servo", group = "TeleOp")
public class Axon_servo extends OpMode {

    private Servo servo1;
    private Servo servo2;
    private double position = 0.5;
    private final double STEP = 0.01;

    @Override
    public void init() {
        servo1 = hardwareMap.get(Servo.class, "servo1");
        servo1.setPosition(position);

        servo2 = hardwareMap.get(Servo.class, "servo2");
        servo2.setPosition(position);

        telemetry.addLine("Servo initialized");
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            position += STEP;
        }

        if (gamepad1.b) {
            position -= STEP;
        }

        if (position > 1.0) position = 1.0;
        if (position < 0.0) position = 0.0;

        servo1.setPosition(position);
        servo2.setPosition(position);

        telemetry.addData("Servo Position", position);
        telemetry.update();
    }
}