package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Shooter Test TeleOp")
public class Shooter extends LinearOpMode {

    private ShooterSubsystem shooter;

    private boolean gateOpen = false;
    private boolean lastX = false;

    @Override
    public void runOpMode() {
        shooter = new ShooterSubsystem(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {

            // Flywheel control
            if (gamepad1.y) {
                shooter.setTargetVelocity(ShooterSubsystem.VELOCITY_HIGH);
            }
            if (gamepad1.a) {
                shooter.setTargetVelocity(ShooterSubsystem.VELOCITY_LOW);
            }
            if (gamepad1.b) {
                shooter.setTargetVelocity(ShooterSubsystem.VELOCITY_IDLE);
            }

            // Intake control
            if (gamepad2.right_bumper) {
                shooter.setIntake(-1.0);
            } else if (gamepad2.left_bumper) {
                shooter.setIntake(1.0);
            } else {
                shooter.stopIntake();
            }

            // Gate toggle
            boolean currentX = gamepad2.x;
            if (currentX && !lastX) {
                gateOpen = !gateOpen;
                shooter.setGateOpen(gateOpen);
            }
            lastX = currentX;

            shooter.periodic();

            telemetry.addLine("=== SHOOTER TEST ===");
            telemetry.addData("Y", "HIGH VELOCITY");
            telemetry.addData("A", "LOW VELOCITY");
            telemetry.addData("B", "STOP FLYWHEEL");
            telemetry.addData("RB", "INTAKE IN");
            telemetry.addData("LB", "INTAKE OUT");
            telemetry.addData("X", "TOGGLE GATE");

            telemetry.addLine();
            telemetry.addData("Gate Open", gateOpen);
            telemetry.addData("Target Velocity", shooter.getTargetVelocity());
            telemetry.addData("Left Velocity", shooter.getLeftVelocity());
            telemetry.addData("Right Velocity", shooter.getRightVelocity());
            telemetry.addData("Average Velocity", shooter.getAverageVelocity());
            telemetry.addData("Applied Power", shooter.getAppliedPower());
            telemetry.addData("At Speed", shooter.isAtSpeed());

            telemetry.update();
        }
    }
}