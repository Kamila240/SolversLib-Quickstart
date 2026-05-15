package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class Intake_limit_withcolorsensor extends LinearOpMode {

    private RevColorSensorV3 sensor1;
    private RevColorSensorV3 sensor2;

    @Override
    public void runOpMode() {
        sensor1 = hardwareMap.get(RevColorSensorV3.class, "sensor1");
        sensor2 = hardwareMap.get(RevColorSensorV3.class, "sensor2");

        DcMotor motor1 = hardwareMap.get(DcMotor.class, "motor1");
        DcMotor motor2 = hardwareMap.get(DcMotor.class, "motor2");

        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        final double SENSOR1_THRESHOLD = 7.0;
        final double SENSOR2_THRESHOLD = 4.0;
        final double INTAKE_POWER = -0.45;

        int ballCount = 0;

        boolean prevSensor1Blocked = false;
        boolean prevSensor2Blocked = false;

        waitForStart();

        while (opModeIsActive()) {

            //boolean intakeStopped;

            boolean intakeStopped = true;
            if (ballCount < 3) {
                intakeStopped = false;
            }
            //boolean intakeStopped = false; } //(ballCount >= 3)
            if (intakeStopped) {
                motor1.setPower(0);
                motor2.setPower(0);
            } else {
                motor1.setPower(INTAKE_POWER);
                motor2.setPower(INTAKE_POWER);
            }

            double distance1 = sensor1.getDistance(DistanceUnit.CM);
            double distance2 = sensor2.getDistance(DistanceUnit.CM);

            boolean sensor1Blocked = distance1 <= SENSOR1_THRESHOLD;
            boolean sensor2Blocked = distance2 <= SENSOR2_THRESHOLD;

            if (!prevSensor1Blocked && sensor1Blocked) {
                if (ballCount < 3) {
                    ballCount++;
                }
            }
            if (ballCount >= 3) {
                intakeStopped = true;
            }

            //ballCount = 0;

            /*if (sensor2Blocked && sensor1Blocked) {
                ballCount = 3;
            }*/

            if (!sensor2Blocked) {//prevSensor2Blocked &&  {
                if (ballCount > 0) {
                    ballCount--;
                    //intakeStopped = false;
                }
            }


            //boolean intakeStopped = (ballCount >= 3); //|| sensor2Blocked

            prevSensor1Blocked = sensor1Blocked;
            prevSensor2Blocked = sensor2Blocked;

            telemetry.addData("Sensor 1 distance", distance1);
            telemetry.addData("Sensor 2 distance", distance2);
            telemetry.addData("Sensor 1 blocked", sensor1Blocked);
            telemetry.addData("Sensor 2 blocked", sensor2Blocked);
            telemetry.addData("ballCount", ballCount);
            telemetry.addData("intakeStopped", intakeStopped);
            telemetry.update();
        }
    }
}