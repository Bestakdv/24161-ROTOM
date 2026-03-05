package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;

@Configurable
@TeleOp(name = "FlywheelTuning")
public class FlywheelTuning extends OpMode {
    private DcMotorEx flywheel;
    private DcMotor coreHex;
    private DcMotor intake;
    private VoltageSensor voltageSensor;

    // TARGETS
    public static double highVelocity = 2300;
    public static double lowVelocity = 1500;


    public static double F = 0;
    public static double P = 0;

    private static final double REFERENCE_VOLTAGE = 12.0;

    double curTargetVelocity = highVelocity;
    boolean lastB = false;

    @Override
    public void init() {
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        coreHex = hardwareMap.get(DcMotor.class, "coreHex");
        intake = hardwareMap.get(DcMotor.class, "intake");
        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setDirection(DcMotorEx.Direction.FORWARD);
    }

    @Override
    public void loop() {
        if (gamepad1.b && !lastB) {
            curTargetVelocity = (curTargetVelocity == highVelocity) ? lowVelocity : highVelocity;
        }
        lastB = gamepad1.b;

        if(gamepad1.x){
            coreHex.setPower(-0.7);
        }
        else if(gamepad1.a){
            intake.setPower(0.7);
        }
        else{
            intake.setPower(0);
            coreHex.setPower(0);
        }

        double currentBatteryVoltage = voltageSensor.getVoltage();
        double voltageMultiplier = REFERENCE_VOLTAGE / currentBatteryVoltage;

        if (currentBatteryVoltage < 5) {
            voltageMultiplier = 1.0;
        }

        double adjustedF = F * voltageMultiplier;

        PIDFCoefficients newPIDF = new PIDFCoefficients(P, 0, 0, adjustedF);
        flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, newPIDF);

        flywheel.setVelocity(curTargetVelocity);

        telemetry.addData("Target Velocity", curTargetVelocity);
        telemetry.addData("Actual Velocity", flywheel.getVelocity());
        telemetry.addLine("-----------------");
        telemetry.addData("Battery Voltage", "%.2f V", currentBatteryVoltage);
        telemetry.addData("Multiplier", "%.2f", voltageMultiplier);
        telemetry.addData("Tuned F", F);
        telemetry.addData("Effective F", "%.2f", adjustedF);
        telemetry.update();
    }
}