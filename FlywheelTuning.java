package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp
@Configurable
public class FlywheelTuning extends OpMode {

    private DcMotorEx flywheel;
    public static double highVelocity = 1900;
    public static double lowVelocity = 1300;
    double curTargetVelocity = highVelocity;
    public static double F=0;
    public static double P=0;

    @Override
    public void init()
    {
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        flywheel.setDirection(DcMotorEx.Direction.REVERSE);
        PIDFCoefficients PIDF = new PIDFCoefficients(P,0,0,F);
        flywheel.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, PIDF);
        telemetry.addLine("Init Complete");
    }

    @Override
    public void loop(){

        if(curTargetVelocity==highVelocity && gamepad1.b){
            curTargetVelocity=lowVelocity;
        }
        if(curTargetVelocity==lowVelocity && gamepad1.b){
            curTargetVelocity=highVelocity;
        }

        PIDFCoefficients PIDF = new PIDFCoefficients(P,0,0,F);
        flywheel.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, PIDF);

        flywheel.setVelocity(curTargetVelocity);
        double curVelocity = flywheel.getVelocity();
        double error = curTargetVelocity - curVelocity;

        telemetry.addData("Target Velocity", curTargetVelocity);
        telemetry.addData("Current Velocity", curVelocity);
        telemetry.addData("Error","%.2f", error);
        telemetry.addLine("--------------------------------");
        telemetry.addData("Tuning P", "%.4f",P);
        telemetry.addData("Tuning F","%.4f", F);

    }
}
