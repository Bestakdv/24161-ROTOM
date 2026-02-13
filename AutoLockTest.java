package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.geometry.Pose;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class AutoLockTest extends OpMode {

    private Follower follower;

    // Target positions
    public double targetX = 6.5;
    public double targetY = 138;

    private boolean autoAim = false;

    private final double MAX_TURN = 0.6;
    //Stops jitter (hopefully)
    private final double DEADBAND_RAD = Math.toRadians(2);

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(16, 118, Math.toRadians(144)));
    }

    @Override
    public void start() {
        follower.startTeleopDrive(true);
        follower.setMaxPower(0.85);
    }

    @Override
    public void loop() {

        follower.update();

        // Toggle auto aim on/off
        if (gamepad1.dpadLeftWasPressed()) {
            autoAim = !autoAim;
        }

        Pose pose = follower.getPose();
        double robotX = pose.getX();
        double robotY = pose.getY();
        double robotHeading = pose.getHeading();

        // Target heading
        double dx = targetX - robotX;
        double dy = targetY - robotY;
        double targetHeading = Math.atan2(dy, dx);

        // Heading error
        double headingError = normalizeAngle(targetHeading - robotHeading);

        //TODO Change controls
        double driveY = -gamepad1.left_stick_y;
        double driveX = -gamepad1.left_stick_x;
        double turn = -gamepad1.right_stick_x;

        // Autoaim
        if (autoAim) {

            if (Math.abs(headingError) > DEADBAND_RAD) {

                // Automatic scaling (no tuning needed)
                turn = headingError / Math.PI;

                // Safety clamp
                turn = clamp(turn, -MAX_TURN, MAX_TURN);

            } else {
                turn = 0;
            }
        }

        follower.setTeleOpDrive(driveY, driveX, turn, true);

        telemetry.addData("Auto Aim Enabled", autoAim);
        telemetry.addData("Heading Error (deg)", Math.toDegrees(headingError));
        telemetry.update();
    }

    // Helper methods

    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    private double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }
}
