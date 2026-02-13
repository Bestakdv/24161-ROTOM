package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

@TeleOp
public class AutoLockTestNoTune extends OpMode {

    private Follower follower;

    // Define motors manually
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor rightBack;

    public double targetX = 6.5;
    public double targetY = 138;

    private boolean autoAim = false;
    private boolean lastDpadLeft = false;

    private final double MAX_TURN_SPEED = 0.7;
    private final double DEADBAND_RAD = Math.toRadians(2.0);

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(16, 118, Math.toRadians(144)));
        
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack   = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack  = hardwareMap.get(DcMotor.class, "rightBack");

        // 3. Set Directions and Zero Power Behavior
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {
        follower.update();
        if (gamepad1.dpad_left && !lastDpadLeft) {
            autoAim = !autoAim;
        }
        lastDpadLeft = gamepad1.dpad_left;

        double forward = -gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x * 1.1;
        double turn = -gamepad1.right_stick_x;
        
        Pose currentPose = follower.getPose();
        
        double dx = targetX - currentPose.getX();
        double dy = targetY - currentPose.getY();
        double targetHeading = Math.atan2(dy, dx);

        double headingError = targetHeading - currentPose.getHeading();
        while (headingError > Math.PI) headingError -= 2 * Math.PI;
        while (headingError < -Math.PI) headingError += 2 * Math.PI;
        
        if (autoAim) {
            if (Math.abs(headingError) > DEADBAND_RAD) {
                double correction = headingError * 1.5;
                turn = Range.clip(correction, -MAX_TURN_SPEED, MAX_TURN_SPEED);
            } else {
                turn = 0;
            }
        }
        
        double denominator = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(turn), 1);
        
        leftFront.setPower((forward + strafe + turn) / denominator);
        leftBack.setPower((forward - strafe + turn) / denominator);
        rightFront.setPower((forward - strafe - turn) / denominator);
        rightBack.setPower((forward + strafe - turn) / denominator);

        // Telemetry
        telemetry.addData("Mode", autoAim ? "AUTO AIM" : "MANUAL");
        telemetry.addData("Error (Deg)", Math.toDegrees(headingError));
        telemetry.addData("Pose", "X:%.1f Y:%.1f H:%.1f", currentPose.getX(), currentPose.getY(), Math.toDegrees(currentPose.getHeading()));
        telemetry.update();
    }
}
