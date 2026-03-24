package org.firstinspires.ftc.teamcode.auton;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.*;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.auton.PoseStorage;

@Autonomous(name = "RedPartnerAuton")
public class RedPartnerAuton extends OpMode {
    private Follower follower;
    private DcMotorEx curry;
    private DcMotor coreHex, intake;
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private VoltageSensor batteryVoltageSensor;

    private final Timer shootTimer = new Timer();
    private final Timer loopTimer = new Timer();
    private final Timer intakeTimer = new Timer();

    private static final double COREHEX_POWER = 1;
    private static final double SHOOT_TIME = 4.5;

    private static final double INTAKE_WAIT_TIME = 0.3;
    private static final double INTAKE_WAIT_TIME1 = 0.5;

    private static final double GOAL_X = 135.5;
    private static final double GOAL_Y = 140;
    public static double MAX_MOTOR_VELOCITY = 1600.0;
    public static double FLYWHEEL_P = 0.8;
    private double targetFlywheelSpeed = 0;

    private final Pose startPose =
            new Pose(90.21160409556313, 6.7713310580204755, Math.toRadians(90));

    private PathChain path1, pathChain2_3, path4,
            pathChain6_7_8, path9, pathChain10_11_12, path13, path14;

    private enum State {
        PATH_1, PATH_1_WAIT, SHOOT_1,
        PATH_2_TO_3, PATH_2_TO_3_WAIT, INTAKE_WAIT_3,
        PATH_4, PATH_4_WAIT, SHOOT_2,
        PATH_6_TO_8, PATH_6_TO_8_WAIT, INTAKE_WAIT_8,
        PATH_9, PATH_9_WAIT, SHOOT_3,
        PATH_10_TO_12, PATH_10_TO_12_WAIT, INTAKE_WAIT_12,
        PATH_13, PATH_13_WAIT, SHOOT_4,
        PATH_14, PATH_14_WAIT,
        DONE
    }

    private State state = State.PATH_1;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        curry = hardwareMap.get(DcMotorEx.class, "flywheel");
        coreHex = hardwareMap.get(DcMotor.class, "coreHex");
        intake = hardwareMap.get(DcMotor.class, "intake");

        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        curry.setDirection(DcMotorEx.Direction.FORWARD);
        coreHex.setDirection(DcMotorEx.Direction.REVERSE);
        intake.setDirection(DcMotorEx.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        curry.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        curry.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        buildPaths();
    }

    @Override
    public void start() {
        intake.setPower(0);
        follower.setMaxPower(1.0);
    }

    @Override
    public void loop() {
        follower.update();

        PoseStorage.currentPose = follower.getPose();

        double autoAimSpeed = flywheelSpeed(getDistanceToGoal());

        switch (state) {
            case PATH_1:
                targetFlywheelSpeed = autoAimSpeed;
                follower.followPath(path1, true);
                state = State.PATH_1_WAIT;
                break;
            case PATH_1_WAIT:
                targetFlywheelSpeed = autoAimSpeed;
                if (!follower.isBusy()) {
                    shootTimer.resetTimer();
                    intakeTimer.resetTimer();
                    state = State.SHOOT_1;
                }
                break;
            case SHOOT_1:
                targetFlywheelSpeed = autoAimSpeed;
                if (shootThree()) state = State.PATH_2_TO_3;
                break;

            case PATH_2_TO_3:
                targetFlywheelSpeed = 0;
                intake.setPower(1);
                follower.followPath(pathChain2_3, true);
                state = State.PATH_2_TO_3_WAIT;
                break;
            case PATH_2_TO_3_WAIT:
                targetFlywheelSpeed = 0;
                if (!follower.isBusy()) {
                    intakeTimer.resetTimer();
                    coreHex.setPower(COREHEX_POWER);
                    state = State.INTAKE_WAIT_3;
                }
                break;
            case INTAKE_WAIT_3:
                targetFlywheelSpeed = 0;
                if (intakeTimer.getElapsedTimeSeconds() >= INTAKE_WAIT_TIME) {
                    coreHex.setPower(0);
                    intake.setPower(0);
                    state = State.PATH_4;
                }
                break;

            case PATH_4:
                targetFlywheelSpeed = autoAimSpeed;
                follower.followPath(path4, true);
                state = State.PATH_4_WAIT;
                break;
            case PATH_4_WAIT:
                targetFlywheelSpeed = autoAimSpeed;
                if (!follower.isBusy()) {
                    shootTimer.resetTimer();
                    intakeTimer.resetTimer();
                    state = State.SHOOT_2;
                }
                break;
            case SHOOT_2:
                targetFlywheelSpeed = autoAimSpeed;
                if (shootThree()) state = State.PATH_6_TO_8;
                break;

            case PATH_6_TO_8:
                targetFlywheelSpeed = 0;
                intake.setPower(1);
                follower.followPath(pathChain6_7_8, true);
                state = State.PATH_6_TO_8_WAIT;
                break;
            case PATH_6_TO_8_WAIT:
                targetFlywheelSpeed = 0;
                if (!follower.isBusy()) {
                    intakeTimer.resetTimer();
                    coreHex.setPower(COREHEX_POWER);
                    state = State.INTAKE_WAIT_8;
                }
                break;
            case INTAKE_WAIT_8:
                targetFlywheelSpeed = 0;
                if (intakeTimer.getElapsedTimeSeconds() >= INTAKE_WAIT_TIME1) {
                    coreHex.setPower(0);
                    intake.setPower(0);
                    state = State.PATH_9;
                }
                break;

            case PATH_9:
                targetFlywheelSpeed = autoAimSpeed;
                follower.followPath(path9, true);
                state = State.PATH_9_WAIT;
                break;
            case PATH_9_WAIT:
                targetFlywheelSpeed = autoAimSpeed;
                if (!follower.isBusy()) {
                    shootTimer.resetTimer();
                    intakeTimer.resetTimer();
                    state = State.SHOOT_3;
                }
                break;
            case SHOOT_3:
                targetFlywheelSpeed = autoAimSpeed;
                if (shootThree()) state = State.PATH_10_TO_12;
                break;

            case PATH_10_TO_12:
                targetFlywheelSpeed = 0;
                intake.setPower(1);
                follower.followPath(pathChain10_11_12, true);
                state = State.PATH_10_TO_12_WAIT;
                break;
            case PATH_10_TO_12_WAIT:
                targetFlywheelSpeed = 0;
                if (!follower.isBusy()) {
                    intakeTimer.resetTimer();
                    coreHex.setPower(COREHEX_POWER);
                    state = State.INTAKE_WAIT_12;
                }
                break;
            case INTAKE_WAIT_12:
                targetFlywheelSpeed = 0;
                if (intakeTimer.getElapsedTimeSeconds() >= INTAKE_WAIT_TIME1) {
                    coreHex.setPower(0);
                    intake.setPower(0);
                    state = State.PATH_13;
                }
                break;

            case PATH_13:
                targetFlywheelSpeed = autoAimSpeed;
                follower.followPath(path13, true);
                state = State.PATH_13_WAIT;
                break;
            case PATH_13_WAIT:
                targetFlywheelSpeed = autoAimSpeed;
                if (!follower.isBusy()) {
                    shootTimer.resetTimer();
                    intakeTimer.resetTimer();
                    state = State.SHOOT_4;
                }
                break;
            case SHOOT_4:
                targetFlywheelSpeed = autoAimSpeed;
                if (shootThree()) state = State.PATH_14;
                break;

            case PATH_14:
                targetFlywheelSpeed = 0;
                follower.followPath(path14, true);
                state = State.PATH_14_WAIT;
                break;
            case PATH_14_WAIT:
                targetFlywheelSpeed = 0;
                if (!follower.isBusy()) state = State.DONE;
                break;

            case DONE:
                targetFlywheelSpeed = 0;
                coreHex.setPower(0);
                intake.setPower(0);
                stopDrive();
                follower.update();
                break;
        }

        setShooterVelocity(targetFlywheelSpeed + 50);

        telemetry.addData("State", state);
        telemetry.addData("Distance", getDistanceToGoal());
        telemetry.addData("Target Speed", targetFlywheelSpeed);
        telemetry.addData("Flywheel Velocity", curry.getVelocity());
        telemetry.update();
    }

    private boolean shootThree() {
        if (shootTimer.getElapsedTimeSeconds() < SHOOT_TIME) {
            if(intakeTimer.getElapsedTimeSeconds() >= 1.5){
                intake.setPower(0.8);
            }

            double currentSpeed = Math.abs(curry.getVelocity());

            if (currentSpeed >= targetFlywheelSpeed+32 && currentSpeed <= targetFlywheelSpeed + 70) {
                coreHex.setPower(COREHEX_POWER);
            } else {
                coreHex.setPower(0);
            }

            return false;
        }

        coreHex.setPower(0);
        targetFlywheelSpeed = 0;
        return true;
    }

    private double getDistanceToGoal() {
        Pose p = follower.getPose();
        return Math.hypot(GOAL_X - p.getX(), GOAL_Y - p.getY());
    }

    private double flywheelSpeed(double x) {
        return MathFunctions.clamp(
                0.0000227176 * Math.pow(x, 4) - 0.0106575 * Math.pow(x, 3) + 1.82035 * Math.pow(x, 2) - 129.17615 * x + 4077.24017,
                0, 2400
        );
    }

    private void setShooterVelocity(double targetVelocity) {
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

    private void stopDrive() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    private void buildPaths() {
        path1 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(90.212, 6.771), new Pose(86.13922093143132, 16.24675388096388)))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(66))
                .build();

        pathChain2_3 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(86.13922093143132, 16.24675388096388), new Pose(98.506, 36.677)))
                .setLinearHeadingInterpolation(Math.toRadians(66), Math.toRadians(0))
                .addPath(new BezierLine(new Pose(98.506, 36.677), new Pose(132.109, 36.637)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        path4 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(132.109, 36.637), new Pose(86.13922093143132, 16.24675388096388)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(66))
                .build();

        pathChain6_7_8 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(86.13922093143132, 16.24675388096388), new Pose(126.885, 14.030)))
                .setLinearHeadingInterpolation(Math.toRadians(66), Math.toRadians(0))
                .addPath(new BezierLine(new Pose(126.885, 14.030), new Pose(127.175, 14.717)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .addPath(new BezierLine(new Pose(127.175, 14.717), new Pose(130.090, 14.886)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        path9 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(130.090, 14.886), new Pose(86.13922093143132, 16.24675388096388)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(66))
                .build();

        pathChain10_11_12 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(86.13922093143132, 16.24675388096388), new Pose(127.114, 14.380)))
                .setLinearHeadingInterpolation(Math.toRadians(66), Math.toRadians(0))
                .addPath(new BezierLine(new Pose(127.114, 14.380), new Pose(130.380, 14.449)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .addPath(new BezierLine(new Pose(130.380, 14.449), new Pose(130.319, 14.778)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        path13 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(130.319, 14.778), new Pose(86.13922093143132, 16.24675388096388)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(66))
                .build();

        path14 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(86.13922093143132, 16.24675388096388), new Pose(100.612, 13.504)))
                .setLinearHeadingInterpolation(Math.toRadians(66), Math.toRadians(66))
                .build();
    }

    @Override
    public void stop() {
        PoseStorage.currentPose = follower.getPose();
    }
}