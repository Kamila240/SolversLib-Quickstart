package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Dual Angle Bottom Dpad Test")
public class AngleSuperSpeedTest extends OpMode {

    private Servo angleLeft;
    private Servo angleRight;

    // Общая позиция механизма:
    // 0.0 = самая нижняя стартовая позиция
    // 1.0 = самая высокая позиция
    private double position = 0.60;

    private static final double STEP = 0.02;

    private static final double MIN_POS = 0.0;
    private static final double MAX_POS = 1.0;

    // Если правая servo стоит зеркально — true.
    // Если обе должны получать одинаковую позицию — false.
    private static final boolean RIGHT_SERVO_REVERSED = true;

    @Override
    public void init() {
        angleLeft = hardwareMap.servo.get("AngleSpeedLeft");
        angleRight = hardwareMap.servo.get("AngleSpeedRight");

        // При INIT сразу ставим в самую нижнюю начальную позицию
        position = MIN_POS;
        setAnglePosition(position);
    }

    @Override
    public void start() {
        // На START тоже фиксируем нижнюю позицию
        position = MIN_POS;
        setAnglePosition(position);
    }

    @Override
    public void loop() {
        if (gamepad2.dpad_up) {
            position += STEP;
        }

        if (gamepad2.dpad_down) {
            position -= STEP;
        }

        // Быстрые пресеты для проверки
        if (gamepad2.x) {
            position = 0.0;   // bottom
        }

        if (gamepad2.a) {
            position = 0.25;
        }

        if (gamepad2.b) {
            position = 0.60;
        }

        if (gamepad2.y) {
            position = 0.75;
        }

        // Безопасно ограничиваем диапазон
        position = clamp(position, MIN_POS, MAX_POS);

        setAnglePosition(position);

        telemetry.addData("Main Position", position);
        telemetry.addData("Left Servo Pos", position);
        telemetry.addData("Right Servo Pos", RIGHT_SERVO_REVERSED ? 1.0 - position : position);
        telemetry.addData("Dpad Up", "raise +0.02");
        telemetry.addData("Dpad Down", "lower -0.02");
        telemetry.addData("X", "bottom 0.00");
        telemetry.addData("A", "0.25");
        telemetry.addData("B", "0.50");
        telemetry.addData("Y", "0.75");
        telemetry.update();
    }

    private void setAnglePosition(double pos) {
        double safePos = clamp(pos, MIN_POS, MAX_POS);

        angleLeft.setPosition(safePos);

        if (RIGHT_SERVO_REVERSED) {
            angleRight.setPosition(1.0 - safePos);
        } else {
            angleRight.setPosition(safePos);
        }
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}