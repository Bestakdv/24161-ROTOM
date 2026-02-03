package org.firstinspires.ftc.teamcode.teleop;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp
public class NewBotTeleopRed extends LinearOpMode {
    private Follower follower;
    private DcMotorEx curry;
    private DcMotor coreHex;
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotor intake;
    private Servo green;
    private GoBildaPinpointDriver pinpoint;

    // TODO CHANGE THESE VALUES BY TESTING THEM WITH BANK BEING CLOSE AND FAR BEING FAR SHOT
    private static final int bankVelocity = 1300;
    private static final int farVelocity = 1900;
    private static final int maxVelocity = 2200;
    private static final double closeHood = 0.4;
    private static final double farHood = 0.6;
    private static double servo_position = 0;
    public static Pose startingPose;
    private boolean manual = false;



    @Override
    public void runOpMode() {
        curry = hardwareMap.get(DcMotorEx.class, "flywheel");
        coreHex = hardwareMap.get(DcMotor.class, "coreHex");
        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft   = hardwareMap.get(DcMotor.class, "backLeft");
        backRight  = hardwareMap.get(DcMotor.class, "backRight");
        intake  = hardwareMap.get(DcMotor.class, "intake");
        green = hardwareMap.get(Servo.class, "hood");
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        MOTOR_SETTINGS();
        pinpoint.recalibrateIMU();
        pinpoint.update();
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                // Calling our methods while the OpMode is running
                mecanumDrive();
                setFlywheelVelocity();
                manualCoreHexAndServoControl();
                telemetry.addData("Flywheel Velocity", ((DcMotorEx) curry).getVelocity());
                telemetry.update();
            }
        }
    }

    private void MOTOR_SETTINGS()
    {
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        curry.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        curry.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        curry.setDirection(DcMotor.Direction.REVERSE);
        coreHex.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        follower = Constants.createFollower(hardwareMap);
        //follower.setStartingPose(startPose);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        PIDFCoefficients PIDF = new PIDFCoefficients(1,0,0,16);
        curry.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, PIDF);
    }

    private void mecanumDrive() {

        double FB = gamepad1.left_stick_y;
        double Strafe = gamepad1.left_stick_x;
        double Turn = -gamepad1.right_stick_x;

        double frontLeftPower = ((FB-Strafe) - Turn);
        double backLeftPower = ((FB+Strafe)-Turn);
        double frontRightPower = ((FB+Strafe) + Turn);
        double backRightPower = ((FB-Strafe) + Turn);

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]

        double[] powers = {
                Math.abs(frontLeftPower),
                Math.abs(frontRightPower),
                Math.abs(backLeftPower),
                Math.abs(backRightPower)
        };

        double max = 0;
        for (double p : powers){
            if (p > max){
                max = p;
            }
        }

        if (max > 1.0) {
            frontLeftPower /= max;
            frontRightPower /= max;
            backLeftPower /= max;
            backRightPower /= max;
        }

        frontLeft.setPower(frontLeftPower);
        backLeft.setPower(backLeftPower);
        frontRight.setPower(frontRightPower);
        backRight.setPower(backRightPower);
    }

    /**
     * Manual control for the Core Hex powered feeder and the agitator servo in the hopper
     */
    private void manualCoreHexAndServoControl() {
        // Manual control for the Core Hex intake
        if (gamepad1.cross) {
            coreHex.setPower(0.7);
        } else if (gamepad1.triangle) {
            coreHex.setPower(-0.7);
        } else if(gamepad1.left_bumper){
            intake.setPower(0.75);
        } else if(gamepad1.dpad_up){
            if(servo_position<1){
                green.setPosition(servo_position+0.05);
                servo_position+=0.05;
            }
        } else if(gamepad1.dpad_down){
            if(servo_position>0){
                green.setPosition(servo_position-0.05);
                servo_position-=0.05;
            }
        }
    }

    private void setFlywheelVelocity() {
        if (gamepad1.options) {
            curry.setPower(-0.5);
            // B on gamepad
        } else if (gamepad1.circle) {
            if(manual){
                curry.setVelocity(farVelocity);
                green.setPosition(farHood);
                servo_position=farHood;}
            else{
                curry.setVelocity(flywheelSpeed(getDistanceToGoal()));
                green.setPosition(hoodAngle(getDistanceToGoal()));
                servo_position=hoodAngle(getDistanceToGoal());
            }
            // X on gamepad
        } else if (gamepad1.square && manual) {
            curry.setVelocity(bankVelocity);
            green.setPosition(closeHood);
            servo_position=closeHood;
        }
        // Y on gamepad
        else if (gamepad1.triangle){
            manual=!manual;
        }
        else {
            ((DcMotorEx) curry).setVelocity(800);
            coreHex.setPower(0);
        }
    }

    private double getDistanceToGoal() {
        Pose p = follower.getPose();
        return Math.hypot(132 - p.getX(), 138 - p.getY());
    }

    // TODO Adjust these
    public static double flywheelSpeed(double x) {
        return MathFunctions.clamp(
                -0.0847688 * x * x + 15.36083 * x + 1225.70392,
                0, 1920
        );
    }

    public static double hoodAngle(double x) {
        return MathFunctions.clamp(
                -0.0847688 * x * x + 15.36083 * x + 1225.70392,
                0, 1
        );
    }

}

