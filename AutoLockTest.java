package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class AutoLockTest extends OpMode {

    private Follower follower;

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
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        follower.update();

        if (gamepad1.dpad_left && !lastDpadLeft) {
            autoAim = !autoAim;
        }
        lastDpadLeft = gamepad1.dpad_left;

        double forward = -gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
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
                turn = headingError * 1.5;
                turn = Math.max(-MAX_TURN_SPEED, Math.min(MAX_TURN_SPEED, turn));
            } else {
                turn = 0;
            }
        }

        follower.setTeleOpDrive(forward, strafe, turn, false);

        telemetry.addData("Auto Aim", autoAim);
        telemetry.addData("Heading Error", Math.toDegrees(headingError));
        telemetry.update();
    }
}
