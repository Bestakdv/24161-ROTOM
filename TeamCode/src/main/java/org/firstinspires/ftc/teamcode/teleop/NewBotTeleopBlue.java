package org.firstinspires.ftc.teamcode.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Vector;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp
@Configurable
public class NewBotTeleopBlue extends LinearOpMode {
    private Follower follower;
    private DcMotorEx curry;
    private DcMotor coreHex;
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotor intake;
    private Servo green;

    public static double velocity = 700;
    private static final int bankVelocity = 1900;
    private static final int farVelocity = 2800;
    private static final double closeHood = 1;
    private static final double farHood = 0.82;
    private static double servo_position = 1;
    private GoBildaPinpointDriver pinpoint;

    // NEED TO TUNE
    public static double SHOT_VELOCITY_IPS = 100.0;
    public static double TURN_KP = 0.5;

    // NEW: Deadband threshold to stop jittering
    public static double TURN_TOLERANCE_DEGREES = 1.5;

    private final Pose GOAL_POSE = new Pose(6.5, 138.0);
    Pose startPoseTest = new Pose(66.075, 6.77, Math.toRadians(90));
    public static Pose startingPose;

    private boolean manual = false;
    private boolean lastDpadLeft = false;

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
                telemetry.addData("Moving Aim Active", gamepad1.right_trigger > 0.1);
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

        curry.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        curry.setDirection(DcMotor.Direction.FORWARD);

        coreHex.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);

        follower = Constants.createFollower(hardwareMap);

        follower.setStartingPose(startPoseTest);
        //follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
    }

    private void mecanumDrive() {
        double FB = -gamepad1.left_stick_y;
        double Strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

        if (gamepad1.right_trigger > 0.1) {
            double targetHeading = getMovingAimAngle(follower, GOAL_POSE, SHOT_VELOCITY_IPS);
            double currentHeading = follower.getPose().getHeading();

            double error = targetHeading - currentHeading;

            while (error > Math.PI) error -= 2 * Math.PI;
            while (error < -Math.PI) error += 2 * Math.PI;

            if (Math.abs(error) < Math.toRadians(TURN_TOLERANCE_DEGREES)) {
                turn = 0;
            } else {
                turn = error * TURN_KP;
            }
            follower.setTeleOpDrive(FB, -Strafe, turn, true);

        } else {
            follower.setTeleOpDrive(0, 0, 0, true);

            double frontLeftPower = (FB + Strafe + turn);
            double backLeftPower = (FB - Strafe + turn);
            double frontRightPower = (FB - Strafe - turn);
            double backRightPower = (FB + Strafe - turn);

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
    }

    private void manualCoreHexAndServoControl() {
        // Cross = Forward, Triangle = Reverse
        if (gamepad1.cross) {
            coreHex.setPower(-0.7);
        } else if (gamepad1.triangle) {
            coreHex.setPower(0.9);
        } else {
            coreHex.setPower(0);
        }

        // Intake
        if(gamepad1.left_bumper) {
            intake.setPower(0.9);
        } else if(gamepad1.right_bumper){
            intake.setPower(-0.9);
        } else {
            intake.setPower(0);
        }
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
                servo_position = farHood;
            }
        }
        else if (gamepad1.square) {
            if(manual){
                curry.setVelocity(bankVelocity);
                servo_position = closeHood;
            }
            else{
                double dist = getDistanceToGoal();
                curry.setVelocity(flywheelSpeed(dist));
                servo_position = closeHood;
            }
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
        return MathFunctions.clamp(
                0.0000223981 * x * x * x * x - 0.0097206 * x * x * x + 1.52224 * x * x - 93.96192 * x + 3493.02631,
                0, 2400
        );
    }

    private double getMovingAimAngle(Follower follower, Pose targetPose, double shotVelocityInchesPerSec) {
        Pose currentPose = follower.getPose();
        Vector velocity = follower.getVelocity();

        double distanceX = targetPose.getX() - currentPose.getX();
        double distanceY = targetPose.getY() - currentPose.getY();
        double distance = Math.hypot(distanceX, distanceY);
        double timeOfFlight = distance / shotVelocityInchesPerSec;
        double virtualX = targetPose.getX() - (velocity.getXComponent() * timeOfFlight);
        double virtualY = targetPose.getY() - (velocity.getYComponent() * timeOfFlight);

        return Math.atan2(virtualY - currentPose.getY(), virtualX - currentPose.getX());
    }
}