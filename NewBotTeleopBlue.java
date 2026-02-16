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
public class NewBotTeleopBlue extends LinearOpMode {
    private Follower follower;
    private DcMotorEx curry;
    private DcMotor coreHex;
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotor intake;
    private Servo green;

    private static final int bankVelocity = 1300;
    private static final int farVelocity = 1900;
    private static final double closeHood = 1;
    private static final double farHood = 0.75;
    private static double servo_position = 1; 
    public static Pose startingPose; 
    private GoBildaPinpointDriver pinpoint;

    private boolean manual = false;
    private boolean lastDpadLeft = false; 
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;

    @Override
    public void runOpMode() {
        // Hardware Mapping
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
        waitForStart();
        
        follower.startTeleopDrive();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                follower.update(); 
                
                mecanumDrive();
                setFlywheelVelocity();
                manualCoreHexAndServoControl();
                
                // Telemetry
                telemetry.addData("Mode", manual ? "MANUAL" : "AUTO-AIM");
                telemetry.addData("Flywheel Velocity", curry.getVelocity());
                telemetry.addData("Target Dist", getDistanceToGoal());
                telemetry.addData("Hood Pos", green.getPosition());
                
                telemetry.addData("X", follower.getPose().getX());
                telemetry.addData("Y", follower.getPose().getY());
                telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
                telemetry.update();
            }
        }
    }

    private void MOTOR_SETTINGS() {
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
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        
        PIDFCoefficients PIDF = new PIDFCoefficients(1,0,0,16);
        curry.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, PIDF);
    }

    private void mecanumDrive() {
        double FB = -gamepad1.left_stick_y; 
        double Strafe = gamepad1.left_stick_x;
        double Turn = -gamepad1.right_stick_x;

        double frontLeftPower = (FB + Strafe + Turn);
        double backLeftPower = (FB - Strafe + Turn);
        double frontRightPower = (FB - Strafe - Turn);
        double backRightPower = (FB + Strafe - Turn);

        double[] powers = { Math.abs(frontLeftPower), Math.abs(frontRightPower), Math.abs(backLeftPower), Math.abs(backRightPower) };
        double max = 0;
        for (double p : powers) if (p > max) max = p;

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

    private void manualCoreHexAndServoControl() {
        // Cross = Forward, Triangle = Reverse
        if (gamepad1.cross) {
            coreHex.setPower(0.7);
        } else if (gamepad1.triangle) { 
            coreHex.setPower(-0.7);
        } else {
            coreHex.setPower(0);
        }

        // Intake
        if(gamepad1.left_bumper) {
            intake.setPower(0.75);
        } else if(gamepad1.right_bumper){
            intake.setPower(-0.4);
        } else {
            intake.setPower(0);
        }

        // Hood Servo
        boolean currentDpadUp = gamepad1.dpad_up;
        boolean currentDpadDown = gamepad1.dpad_down;

        if (currentDpadUp && !lastDpadUp) {
            if (servo_position < 1) servo_position += 0.05;
        }
        if (currentDpadDown && !lastDpadDown) {
            if (servo_position > 0) servo_position -= 0.05;
        }
        
        lastDpadUp = currentDpadUp;
        lastDpadDown = currentDpadDown;
    }

    private void setFlywheelVelocity() {
        // Manual Mode Toggle
        boolean currentDpadLeft = gamepad1.dpad_left;
        if (currentDpadLeft && !lastDpadLeft) {
            manual = !manual;
        }
        lastDpadLeft = currentDpadLeft;

        if (gamepad1.options) {
            curry.setPower(-0.5);
        } 
        else if (gamepad1.circle) {
             if(manual){
                curry.setVelocity(farVelocity);
                servo_position = farHood;
             } else {
                double dist = getDistanceToGoal();
                curry.setVelocity(flywheelSpeed(dist));
                servo_position = hoodAngle(dist);
             }
        } 
        else if (gamepad1.square) { 
            curry.setVelocity(bankVelocity);
            servo_position = closeHood;
        } 
        else {
            curry.setVelocity(0);
        }
        green.setPosition(servo_position);
    }

    private double getDistanceToGoal() {
        Pose p = follower.getPose();
        return Math.hypot(6.5 - p.getX(), 138 - p.getY());
    }

    public static double flywheelSpeed(double x) {
        //TODO CHANGE
        return MathFunctions.clamp(
                -0.0847688 * x * x + 15.36083 * x + 1225.70392,
                0, 2200
        );
    }

    public static double hoodAngle(double x) {
        //TODO CHANGE
        return 0.5; 
    }
}
