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
public class NewBotTeleopRedTest extends OpMode {
    private Follower follower;
    public double targetX = 138.0;
    public double targetY = 138.0;
    private boolean autoAimEngaged = false;
    private boolean lastRightTrigger = false;
    private boolean manual = false;
    private boolean lastDpadLeft = false;
    private VoltageSensor batteryVoltageSensor;
    private DcMotorEx curry;
    private DcMotor coreHex;
    private DcMotor intake;
    public static double velocity = 700;
    private static final int bankVelocity = 1900;
    private static final int farVelocity = 2400;

    //TODO TUNE THIS FOR YOUR FLYWHEEL MAXIMUM VELOCITY CHANGE TO PRIVATE WHEN DONE
    public static final double MAX_MOTOR_VELOCITY = 2020.0;
    //TODO TUNE THIS FOR LEFT STRAFE AND RIGHT FOR STRAFING RIGHT CHANGE TO PRIVATE WHEN DONE
    public static double MOTION_COMPENSATION = 0.022;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(66.075, 7.017, Math.toRadians(90)));
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

        double vTargetX = targetX - (vx * distance * MOTION_COMPENSATION);
        double vTargetY = targetY - (vy * distance * MOTION_COMPENSATION);

        double dx = vTargetX - follower.getPose().getX();
        double dy = vTargetY - follower.getPose().getY();

        double targetHeading = Math.atan2(dy, dx);
        double headingerror = Math.toDegrees(targetHeading) - Math.toDegrees(follower.getPose().getHeading());

        while (headingerror > 180.0)  headingerror -= 360.0;
        while (headingerror <= -180.0) headingerror += 360.0;

        if (autoAimEngaged) {
            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    headingerror / 100.0,
                    true
            );
        } else {
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
                0.0000223981 * x * x * x * x - 0.0097206 * x * x * x + 1.52224 * x * x - 93.96192 * x + 3493.02631,
                0, 2400
        );
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
        double compensatedPower = basePower * (12.0 / currentVoltage);

        curry.setPower(Math.min(compensatedPower, 1.0));
    }
}