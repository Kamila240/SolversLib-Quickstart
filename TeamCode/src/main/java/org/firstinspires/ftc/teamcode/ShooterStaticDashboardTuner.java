package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

@Config
@TeleOp(name = "Shooter Static Dashboard Tuner")
public class ShooterStaticDashboardTuner extends OpMode {

    private DcMotorEx flywheelL;
    private DcMotorEx flywheelR;

    private DcMotorEx intake1;
    private DcMotorEx intake2;

    private Servo stopper;
    private VoltageSensor batterySensor;

    private FtcDashboard dashboard;

    // ============================
    // Hardware names
    // ============================
    private static final String LEFT_SHOOTER_NAME = "shooter2";
    private static final String RIGHT_SHOOTER_NAME = "shooter1";

    private static final String INTAKE1_NAME = "intake1";
    private static final String INTAKE2_NAME = "intake2";

    private static final String STOPPER_NAME = "stopper";

    // ============================
    // Dashboard controls
    // ============================
    public static boolean SHOOTER_ON = false;
    public static boolean USE_GAMEPAD = true;

    public static double TARGET_VELOCITY = 1700.0;

    // Feedforward + PID
    public static double kV = 0.0004005;
    public static double kS = 0.105;
    public static double kP = 0.0010;
    public static double kD = 0.0;

    // Voltage compensation
    public static boolean USE_VOLTAGE_COMP = true;
    public static double REFERENCE_VOLTAGE = 12.0;

    // Spin-up boost
    public static boolean USE_SPINUP_BOOST = true;
    public static double SPINUP_BOOST_RATIO = 0.85;

    public static double VELOCITY_FILTER_ALPHA = 0.35;

    // Fixed power mode
    public static boolean USE_FIXED_POWER = false;
    public static boolean USE_VOLTAGE_COMP_IN_FIXED_POWER = false;
    public static double FIXED_POWER = 0.70;

    // Stopper
    public static double STOPPER_CLOSED = 0.5;
    public static double STOPPER_OPEN = 0.7;
    public static boolean STOPPER_OPEN_DASHBOARD = false;

    // Intake
    public static boolean INTAKE_ON_DASHBOARD = false;
    public static boolean INTAKE_REVERSE_DASHBOARD = false;

    public static double INTAKE_POWER = 1.0;
    public static double INTAKE_REVERSE_POWER = 0.6;

    // Motor directions
    public static boolean REVERSE_LEFT_SHOOTER = true;
    public static boolean REVERSE_RIGHT_SHOOTER = false;

    public static boolean REVERSE_INTAKE1 = false;
    public static boolean REVERSE_INTAKE2 = false;

    // Safety
    public static double MAX_POWER = 1.0;

    private double measuredVelocity = 0.0;
    private double filteredVelocity = 0.0;
    private double leftVelocity = 0.0;
    private double rightVelocity = 0.0;

    private double shooterPower = 0.0;
    private double intakePower = 0.0;

    private double error = 0.0;
    private double lastError = 0.0;
    private double lastTimeSec = 0.0;

    private double ffTerm = 0.0;
    private double pTerm = 0.0;
    private double dTerm = 0.0;

    private double rawPower = 0.0;
    private double compensatedPower = 0.0;

    private boolean filterInitialized = false;
    private boolean spinupBoostActive = false;

    @Override
    public void init() {
        dashboard = FtcDashboard.getInstance();

        telemetry = new MultipleTelemetry(
                telemetry,
                dashboard.getTelemetry()
        );

        flywheelL = hardwareMap.get(DcMotorEx.class, LEFT_SHOOTER_NAME);
        flywheelR = hardwareMap.get(DcMotorEx.class, RIGHT_SHOOTER_NAME);

        intake1 = hardwareMap.get(DcMotorEx.class, INTAKE1_NAME);
        intake2 = hardwareMap.get(DcMotorEx.class, INTAKE2_NAME);

        stopper = hardwareMap.get(Servo.class, STOPPER_NAME);

        batterySensor = hardwareMap.voltageSensor.iterator().next();

        flywheelL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheelR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        flywheelL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheelR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        intake1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intake1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        applyDirections();

        stopper.setPosition(STOPPER_CLOSED);

        setFlywheelPower(0.0);
        setIntakePower(0.0);

        resetShooterMath();

        lastTimeSec = getRuntime();

        telemetry.addLine("Shooter Static Dashboard Tuner Ready");
        telemetry.addLine("Logic: FF + PID + voltage compensation + spin-up boost");
        telemetry.addLine("Intake motors: intake1 + intake2");
        telemetry.update();
    }

    @Override
    public void loop() {
        applyDirections();

        double now = getRuntime();
        double dt = Math.max(0.001, now - lastTimeSec);
        lastTimeSec = now;

        updateVelocity();

        // ============================
        // Shooter control
        // ============================
        boolean gamepadShooterOn = USE_GAMEPAD && gamepad2.right_trigger > 0.5;
        boolean gamepadShooterOff = USE_GAMEPAD && gamepad2.left_trigger > 0.5;

        boolean shouldRunShooter = SHOOTER_ON || gamepadShooterOn;

        if (gamepadShooterOff) {
            shouldRunShooter = false;
            SHOOTER_ON = false;
        }

        if (shouldRunShooter) {
            if (USE_FIXED_POWER) {
                rawPower = Range.clip(FIXED_POWER, 0.0, MAX_POWER);

                if (USE_VOLTAGE_COMP_IN_FIXED_POWER) {
                    compensatedPower = compensateShooterPower(rawPower);
                    shooterPower = compensatedPower;
                } else {
                    compensatedPower = rawPower;
                    shooterPower = rawPower;
                }

                ffTerm = 0.0;
                pTerm = 0.0;
                dTerm = 0.0;
                error = TARGET_VELOCITY - filteredVelocity;
                spinupBoostActive = false;
            } else {
                shooterPower = calculateShooterPower(filteredVelocity, dt);
            }

            setFlywheelPower(shooterPower);
        } else {
            setFlywheelPower(0.0);
            resetShooterMath();
            filterInitialized = false;
            lastTimeSec = getRuntime();
        }

        // ============================
        // Stopper control
        // ============================
        if (STOPPER_OPEN_DASHBOARD || (USE_GAMEPAD && gamepad2.right_bumper)) {
            stopper.setPosition(STOPPER_OPEN);
        } else {
            stopper.setPosition(STOPPER_CLOSED);
        }

        if (USE_GAMEPAD && gamepad2.left_bumper) {
            STOPPER_OPEN_DASHBOARD = false;
            stopper.setPosition(STOPPER_CLOSED);
        }

        // ============================
        // Intake control
        // gamepad2.a = forward
        // gamepad2.b = reverse
        // gamepad2.x = stop dashboard intake
        // ============================
        boolean intakeForward =
                INTAKE_ON_DASHBOARD ||
                        (USE_GAMEPAD && gamepad2.a);

        boolean intakeReverse =
                INTAKE_REVERSE_DASHBOARD ||
                        (USE_GAMEPAD && gamepad2.b);

        if (USE_GAMEPAD && gamepad2.x) {
            INTAKE_ON_DASHBOARD = false;
            INTAKE_REVERSE_DASHBOARD = false;
        }

        if (intakeReverse) {
            intakePower = -Range.clip(INTAKE_REVERSE_POWER, 0.0, 1.0);
        } else if (intakeForward) {
            intakePower = Range.clip(INTAKE_POWER, 0.0, 1.0);
        } else {
            intakePower = 0.0;
        }

        setIntakePower(intakePower);

        double batteryVoltage = getBatteryVoltage();
        double voltageRatio = REFERENCE_VOLTAGE / batteryVoltage;
        boolean shooterReady = filteredVelocity >= TARGET_VELOCITY - 120.0;

        // ============================
        // Dashboard graph
        // ============================
        TelemetryPacket packet = new TelemetryPacket();

        packet.put("targetVelocity", TARGET_VELOCITY);
        packet.put("measuredVelocity", measuredVelocity);
        packet.put("filteredVelocity", filteredVelocity);
        packet.put("leftVelocity", leftVelocity);
        packet.put("rightVelocity", rightVelocity);

        packet.put("error", error);
        packet.put("power", shooterPower);
        packet.put("rawPower", rawPower);
        packet.put("compensatedPower", compensatedPower);

        packet.put("ffTerm", ffTerm);
        packet.put("pTerm", pTerm);
        packet.put("dTerm", dTerm);

        packet.put("spinupBoostActive", spinupBoostActive);
        packet.put("shooterReady", shooterReady);

        packet.put("intakePower", intakePower);
        packet.put("batteryVoltage", batteryVoltage);
        packet.put("voltageRatio", voltageRatio);

        dashboard.sendTelemetryPacket(packet);

        telemetry.addData("Shooter ON", shouldRunShooter);
        telemetry.addData("Target Velocity", TARGET_VELOCITY);

        telemetry.addData("Measured Velocity", measuredVelocity);
        telemetry.addData("Filtered Velocity", filteredVelocity);
        telemetry.addData("Left Velocity", leftVelocity);
        telemetry.addData("Right Velocity", rightVelocity);

        telemetry.addData("Shooter Ready", shooterReady);
        telemetry.addData("Error", error);

        telemetry.addData("Raw Power", rawPower);
        telemetry.addData("Compensated Power", compensatedPower);
        telemetry.addData("Applied Power", shooterPower);

        telemetry.addData("FF Term", ffTerm);
        telemetry.addData("P Term", pTerm);
        telemetry.addData("D Term", dTerm);

        telemetry.addData("Spinup Boost Active", spinupBoostActive);
        telemetry.addData("Spinup Boost Ratio", SPINUP_BOOST_RATIO);

        telemetry.addData("Intake 1 Name", INTAKE1_NAME);
        telemetry.addData("Intake 2 Name", INTAKE2_NAME);
        telemetry.addData("Intake Power", intakePower);
        telemetry.addData("Reverse Intake 1", REVERSE_INTAKE1);
        telemetry.addData("Reverse Intake 2", REVERSE_INTAKE2);

        telemetry.addData("Stopper Pos", STOPPER_OPEN_DASHBOARD ? STOPPER_OPEN : STOPPER_CLOSED);

        telemetry.addData("Battery Voltage", batteryVoltage);
        telemetry.addData("Reference Voltage", REFERENCE_VOLTAGE);
        telemetry.addData("Voltage Ratio", voltageRatio);
        telemetry.addData("Voltage Comp", USE_VOLTAGE_COMP);

        telemetry.addData("kV", kV);
        telemetry.addData("kS", kS);
        telemetry.addData("kP", kP);
        telemetry.addData("kD", kD);

        telemetry.update();
    }

    @Override
    public void stop() {
        setFlywheelPower(0.0);
        setIntakePower(0.0);
        stopper.setPosition(STOPPER_CLOSED);
        resetShooterMath();
    }

    private void updateVelocity() {
        leftVelocity = Math.abs(flywheelL.getVelocity());
        rightVelocity = Math.abs(flywheelR.getVelocity());

        measuredVelocity = (leftVelocity + rightVelocity) / 2.0;

        if (!filterInitialized) {
            filteredVelocity = measuredVelocity;
            filterInitialized = true;
        } else {
            filteredVelocity += VELOCITY_FILTER_ALPHA * (measuredVelocity - filteredVelocity);
        }
    }

    private double calculateShooterPower(double velocity, double dt) {
        error = TARGET_VELOCITY - velocity;

        double derivative = (error - lastError) / dt;
        lastError = error;

        ffTerm = kV * TARGET_VELOCITY + kS * Math.signum(TARGET_VELOCITY);
        pTerm = kP * error;
        dTerm = kD * derivative;

        rawPower = ffTerm + pTerm + dTerm;

        spinupBoostActive = false;

        if (USE_SPINUP_BOOST && velocity < TARGET_VELOCITY * SPINUP_BOOST_RATIO) {
            rawPower = 1.0;
            spinupBoostActive = true;
        }

        compensatedPower = compensateShooterPower(rawPower);

        return compensatedPower;
    }

    private double compensateShooterPower(double power) {
        if (!USE_VOLTAGE_COMP) {
            return Range.clip(power, 0.0, MAX_POWER);
        }

        double voltage = getBatteryVoltage();
        double compensated = power * (REFERENCE_VOLTAGE / voltage);

        return Range.clip(compensated, 0.0, MAX_POWER);
    }

    private void resetShooterMath() {
        shooterPower = 0.0;
        rawPower = 0.0;
        compensatedPower = 0.0;

        ffTerm = 0.0;
        pTerm = 0.0;
        dTerm = 0.0;

        error = 0.0;
        lastError = 0.0;

        spinupBoostActive = false;
    }

    private void applyDirections() {
        flywheelL.setDirection(
                REVERSE_LEFT_SHOOTER ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD
        );

        flywheelR.setDirection(
                REVERSE_RIGHT_SHOOTER ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD
        );

        intake1.setDirection(
                REVERSE_INTAKE1 ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD
        );

        intake2.setDirection(
                REVERSE_INTAKE2 ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD
        );
    }

    private void setFlywheelPower(double power) {
        flywheelL.setPower(power);
        flywheelR.setPower(power);
    }

    private void setIntakePower(double power) {
        intake1.setPower(power);
        intake2.setPower(power);
    }

    private double getBatteryVoltage() {
        double voltage = batterySensor.getVoltage();
        return voltage > 0.0 ? voltage : REFERENCE_VOLTAGE;
    }
}