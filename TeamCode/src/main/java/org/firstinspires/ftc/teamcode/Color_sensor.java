package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class Color_sensor extends LinearOpMode {

    private RevColorSensorV3 sensor1;
    private RevColorSensorV3 sensor2;

    @Override
    public void runOpMode() {
        sensor1 = hardwareMap.get(RevColorSensorV3.class, "sensor1");
        sensor2 = hardwareMap.get(RevColorSensorV3.class, "sensor2");

        waitForStart();

        while (opModeIsActive()) {
            double distance1 = sensor1.getDistance(DistanceUnit.CM);
            double distance2 = sensor2.getDistance(DistanceUnit.CM);

            telemetry.addData("Sensor 1 distance (cm)", distance1);
            telemetry.addData("Sensor 2 distance (cm)", distance2);

            telemetry.update();
        }
    }
}