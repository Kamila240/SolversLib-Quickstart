package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Stopper Servo Test")
public class StopperServoTest extends OpMode {

    private Servo stopper;

    private double position = 0.50;

    private static final double STEP = 0.01;
    private static final double MIN_POS = 0.0;
    private static final double MAX_POS = 1.0;

    @Override
    public void init() {
        stopper = hardwareMap.servo.get("stopper");

        // Начальная позиция при INIT
        position = 0.50;
        stopper.setPosition(position);
    }

    @Override
    public void loop() {
        if (gamepad2.dpad_up) {
            position += STEP;
        }

        if (gamepad2.dpad_down) {
            position -= STEP;
        }

        // Быстрые позиции для теста
        if (gamepad2.x) {
            position = 0.30;
        }

        if (gamepad2.a) {
            position = 0.50;
        }

        if (gamepad2.y) {
            position = 0.70;
        }

        if (gamepad2.b) {
            position = 0.90;
        }

        position = clamp(position, MIN_POS, MAX_POS);

        stopper.setPosition(position);

        telemetry.addData("Stopper Position", position);
        telemetry.addData("Dpad Up", "+0.01");
        telemetry.addData("Dpad Down", "-0.01");
        telemetry.addData("X", "0.30");
        telemetry.addData("A", "0.50");
        telemetry.addData("Y", "0.70");
        telemetry.addData("B", "0.90");
        telemetry.update();
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}