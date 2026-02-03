package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp
@Configurable
public class DataForTeleop extends LinearOpMode {
    private DcMotorEx flywheel;
    private DcMotor coreHex;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    public static int speedVelocity = 1750;
    private Follower follower;
    Pose startPose = new Pose(27.081,133.973,Math.toRadians(144));
    //public static Pose startingPose;
    private double distanceToGoal;

    @Override
    public void runOpMode() {
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        coreHex = hardwareMap.get(DcMotor.class, "coreHex");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        MOTOR_SETTINGS();
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                // Calling our methods while the OpMode is running
                mecanumDrive();
                setFlywheelVelocity();
                manualCoreHexAndServoControl();
                telemetry.addData("Flywheel Velocity", ((DcMotorEx) flywheel).getVelocity());
                telemetry.addData("Distance From Goal", distanceToGoal);
                telemetry.update();
            }
        }
    }

    private void MOTOR_SETTINGS()
    {
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel.setDirection(DcMotor.Direction.REVERSE);
        coreHex.setDirection(DcMotor.Direction.REVERSE);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        PIDFCoefficients PIDF = new PIDFCoefficients(1,0,0,16);
        flywheel.setPIDFCoefficients(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER, PIDF);
        //follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
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

        follower.update();
        double dx = 13 - follower.getPose().getX();
        double dy = 138 - follower.getPose().getY();
        distanceToGoal = Math.hypot(dx,dy);
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
        }
        else{
            coreHex.setPower(0);
        }
    }

    private void setFlywheelVelocity() {
       if (gamepad1.right_bumper) {
           ((DcMotorEx) flywheel).setVelocity(speedVelocity);
        }
       else {
           ((DcMotorEx) flywheel).setVelocity(0);
       }
    }

}
