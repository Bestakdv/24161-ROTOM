package org.firstinspires.ftc.teamcode.teleop;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@TeleOp
public class NewBotTeleopBlueTest extends OpMode {
    private Follower follower;
    public double targetX = 10;
    public double targetY = 140;
    private boolean autoAimEngaged = false;
    private boolean lastRightTrigger = false;
    private boolean manual = false;
    private boolean lastDpadLeft = false;
    private VoltageSensor batteryVoltageSensor;
    private DcMotorEx curry;
    private DcMotor coreHex;
    private DcMotor intake;
    public static int bankVelocity = 1000;
    public static int farVelocity = 1200;

    public static double FLYWHEEL_P = 0.8;

    public static double MAX_MOTOR_VELOCITY = 1600;

    public static double MOTION_COMP_RIGHT = 0.021;
    public static double MOTION_COMP_LEFT = 0.021;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(66.075, 7.017, Math.toRadians(90)));
        //TODO UNCOMMENT LINE BELOW WHEN USING AUTON
        //follower.setStartingPose(startingPose == null ? new Pose() : startingPose);

        curry = hardwareMap.get(DcMotorEx.class, "flywheel");
        coreHex = hardwareMap.get(DcMotor.class, "coreHex");
        intake  = hardwareMap.get(DcMotor.class, "intake");
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        curry.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        curry.setDirection(DcMotor.Direction.FORWARD);
        coreHex.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    @Override
    public void start() {
        follower.startTeleopDrive(true);
        follower.setMaxPower(1);
    }

    @Override
    public void loop() {
        boolean currentRightTrigger = gamepad1.right_trigger > 0.1;
        if (currentRightTrigger && !lastRightTrigger) {
            autoAimEngaged = !autoAimEngaged;
        }
        lastRightTrigger = currentRightTrigger;

        follower.update();

        Vector currentVelocity = follower.getVelocity();
        double vx = currentVelocity.getXComponent();
        double vy = currentVelocity.getYComponent();

        double distance = Math.hypot(targetX - follower.getPose().getX(), targetY - follower.getPose().getY());

        // Calculate the raw vector to the target to determine our orientation
        double dx_raw = targetX - follower.getPose().getX();
        double dy_raw = targetY - follower.getPose().getY();

        // Cross product determines if velocity is drifting left or right of the target
        double crossProduct = (vx * dy_raw) - (vy * dx_raw);

        // Positive cross product means right strafe relative to goal, negative means left
        double activeMotionComp = (crossProduct > 0) ? MOTION_COMP_RIGHT : MOTION_COMP_LEFT;

        double vTargetX = targetX - (vx * distance * activeMotionComp);
        double vTargetY = targetY - (vy * distance * activeMotionComp);

        double dx = vTargetX - follower.getPose().getX();
        double dy = vTargetY - follower.getPose().getY();

        double targetHeading = Math.atan2(dy, dx);
        double headingerror = Math.toDegrees(targetHeading) - Math.toDegrees(follower.getPose().getHeading());

        while (headingerror > 180.0)  headingerror -= 360.0;
        while (headingerror <= -180.0) headingerror += 360.0;

        if (autoAimEngaged) {
            follower.setMaxPower(0.7);
            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    headingerror / 130.0,
                    true
            );
        } else {
            follower.setMaxPower(1);
            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y * 1.1,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    true
            );
        }
        manualCoreHexAndServoControl();
        setFlywheelVelocity();

        telemetry.addData("Aim Mode", autoAimEngaged ? "AUTO LOCK" : "MANUAL");
        telemetry.addData("Flywheel Mode", manual ? "MANUAL" : "AUTO-SPEED");
        telemetry.addData("Active Motion Comp", crossProduct > 0 ? "RIGHT (" + MOTION_COMP_RIGHT + ")" : "LEFT (" + MOTION_COMP_LEFT + ")");
        telemetry.addData("Target Heading Error Blue:", headingerror);
        telemetry.addData("Flywheel Velocity", curry.getVelocity());
        telemetry.addData("Distance", getDistanceToGoal());
        telemetry.update();
    }


    private void manualCoreHexAndServoControl() {
        if (gamepad1.cross) {
            coreHex.setPower(-0.7);
        } else if (gamepad1.triangle) {
            coreHex.setPower(0.9);
        } else {
            coreHex.setPower(0);
        }

        if (gamepad1.left_bumper) {
            intake.setPower(0.9);
        } else if (gamepad1.right_bumper) {
            intake.setPower(-0.9);
        } else {
            intake.setPower(0);
        }
    }

    private void setFlywheelVelocity() {
        boolean currentDpadLeft = gamepad1.dpad_left;
        if (currentDpadLeft && !lastDpadLeft) {
            manual = !manual;
        }
        lastDpadLeft = currentDpadLeft;

        if (gamepad1.options) {
            curry.setPower(-0.5);
        }
        else if (gamepad1.circle) {
            if (manual) {
                setFlywheelVelocityCustom(farVelocity);
            } else {
                double dist = getDistanceToGoal();
                setFlywheelVelocityCustom(flywheelSpeed(dist));
            }
        }
        else if (gamepad1.square) {
            if (manual) {
                setFlywheelVelocityCustom(bankVelocity);
            } else {
                double dist = getDistanceToGoal();
                setFlywheelVelocityCustom(flywheelSpeed(dist));
            }
        }
        else {
            setFlywheelVelocityCustom(0);
        }
    }

    private double getDistanceToGoal() {
        Pose p = follower.getPose();
        return Math.hypot(targetX - p.getX(), targetY - p.getY());
    }

    public static double flywheelSpeed(double x) {
        return MathFunctions.clamp(
                0.0000227176 * Math.pow(x,4) - 0.0106575 * Math.pow(x,3) + 1.82035 * Math.pow(x,2)- 129.17615 * x + 4077.24017,
                0, 2400
        ) ;
    }

    private void setFlywheelVelocityCustom(double targetVelocity) {
        if (targetVelocity == 0) {
            curry.setPower(0);
            return;
        }

        double currentVoltage = batteryVoltageSensor.getVoltage();
        if (currentVoltage <= 0) {
            currentVoltage = 12.0;
        }

        double basePower = targetVelocity / MAX_MOTOR_VELOCITY;
        double feedforward = basePower * (12.0 / currentVoltage);

        double currentSpeed = curry.getVelocity();
        double error = targetVelocity - currentSpeed;
        double feedback = error * FLYWHEEL_P;

        double totalPower = feedforward + feedback;
        curry.setPower(Math.max(0.0, Math.min(totalPower, 1.0)));
    }
}
