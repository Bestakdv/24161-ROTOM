package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import com.pedropathing.math.MathFunctions;

import java.util.List;

@TeleOp
@Configurable
public class AutoLockTestLL extends OpMode {
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor rightBack;

    // --- Hardware Subsystem Variables ---
    private DcMotorEx curry;
    private DcMotor coreHex;
    private DcMotor intake;
    private Servo green;

    public static double velocity = 700;
    private static final int bankVelocity = 1900;
    private static final int farVelocity = 2800; // Consider lowering if you change pulleys
    private static final double closeHood = 1;
    private static final double farHood = 0.82;
    private static double servo_position = 1;
    private boolean manual = false;
    private boolean lastDpadLeft = false;

    private final LimelightHelper limelightHelper = new LimelightHelper();

    // PD stuff for Limelight rotation
    public static double kP = 0.025;
    double error = 0;
    double lastError = 0;
    double goalX = 0;
    double angleTolerance = 0.4;
    public static double kD = 0.002;
    double curTime = 0;
    double lastTime = 0;

    // Drive stuff
    double forward, strafe, rotate;

    @Override
    public void init(){
        // Initialize Vision
        limelightHelper.init(hardwareMap);

        // --- Initialize Drive Motors ---
        leftFront = hardwareMap.get(DcMotor.class, "frontLeft");
        rightFront = hardwareMap.get(DcMotor.class, "frontRight");
        leftBack = hardwareMap.get(DcMotor.class, "backLeft");
        rightBack = hardwareMap.get(DcMotor.class, "backRight");

        // --- Initialize Subsystem Motors/Servos ---
        curry = hardwareMap.get(DcMotorEx.class, "flywheel");
        coreHex = hardwareMap.get(DcMotor.class, "coreHex");
        intake  = hardwareMap.get(DcMotor.class, "intake");
        green = hardwareMap.get(Servo.class, "hood");

        // --- Motor Directions ---
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);

        curry.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        curry.setDirection(DcMotor.Direction.FORWARD);
        coreHex.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);

        // --- Zero Power Behavior ---
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void start(){
        resetRuntime();
        curTime = getRuntime();
    }

    @Override
    public void loop(){
        forward = -gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        rotate = gamepad1.right_stick_x;

        LLResult result = limelightHelper.getLatestResult();
        Double targetTx = null;
        Double targetZ = null;

        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            for (LLResultTypes.FiducialResult tag : fiducials) {
                if (tag.getFiducialId() == 20) {
                    Pose3D tagPose = tag.getCameraPoseTargetSpace();

                    double x = tagPose.getPosition().x;
                    double z = tagPose.getPosition().z;

                    targetTx = Math.toDegrees(Math.atan2(x, z));
                    targetZ = z;
                    break;
                }
            }
        }

        boolean autoAimActive = gamepad1.right_trigger > 0.1 || gamepad1.left_trigger > 0.1;
        if(autoAimActive){
            if(targetTx != null){
                error = goalX - targetTx;
                if(Math.abs(error) < angleTolerance){
                    rotate = 0;
                }
                else{
                    double pTerm = error * kP;
                    curTime = getRuntime();
                    double dT = curTime - lastTime;
                    if (dT == 0) dT = 0.001;

                    double dTerm = ((error-lastError)/dT) * kD;
                    rotate = Range.clip(pTerm + dTerm, -0.5, 0.5);

                    lastError = error;
                    lastTime = curTime;
                }
            } else{
                rotate = 0;
                lastTime = getRuntime();
                lastError = 0;
            }
        } else {
            lastError = 0;
            lastTime = getRuntime();
        }
        double denominator = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(rotate), 1);

        double leftFrontPower  = (forward + strafe + rotate) / denominator;
        double rightFrontPower = (forward - strafe - rotate) / denominator;
        double leftBackPower   = (forward - strafe + rotate) / denominator;
        double rightBackPower  = (forward + strafe - rotate) / denominator;

        leftFront.setPower(leftFrontPower);
        rightFront.setPower(rightFrontPower);
        leftBack.setPower(leftBackPower);
        rightBack.setPower(rightBackPower);

        manualCoreHexAndServoControl();
        setFlywheelVelocity(targetZ);

        telemetry.addData("ID 20 Found", targetTx != null);
        telemetry.addData("Calculated tx", targetTx);
        telemetry.addData("Aim Mode", autoAimActive ? "LIMELIGHT AUTO LOCK" : "MANUAL");
        telemetry.addData("Distance to Tag (Z)", targetZ);
        telemetry.addData("Flywheel Velocity", curry.getVelocity());
        telemetry.update();
    }

    @Override
    public void stop() {
        limelightHelper.stop();
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
        if (gamepad1.left_bumper) {
            intake.setPower(0.9);
        } else if (gamepad1.right_bumper) {
            intake.setPower(-0.9);
        } else {
            intake.setPower(0);
        }
    }

    private void setFlywheelVelocity(Double currentDistanceZ) {
        boolean currentDpadLeft = gamepad1.dpad_left;
        if (currentDpadLeft && !lastDpadLeft) {
            manual = !manual;
        }
        lastDpadLeft = currentDpadLeft;

        if (gamepad1.options) {
            curry.setPower(-0.5);
        }
        else if (gamepad1.circle) {
            if (manual || currentDistanceZ == null) {
                curry.setVelocity(farVelocity);
                servo_position = farHood;
            } else {
                curry.setVelocity(flywheelSpeed(currentDistanceZ * 39.37));
                servo_position = farHood;
            }
        }
        else if (gamepad1.square) {
            if (manual || currentDistanceZ == null) {
                curry.setVelocity(bankVelocity);
                servo_position = closeHood;
            } else {
                curry.setVelocity(flywheelSpeed(currentDistanceZ * 39.37));
                servo_position = closeHood;
            }
        }
        else {
            curry.setVelocity(0);
        }
        green.setPosition(servo_position);
    }

    // You will likely need to tune this polynomial again if the input units (meters vs inches) change!
    public static double flywheelSpeed(double x) {
        return MathFunctions.clamp(
                0.0000223981 * x * x * x * x - 0.0097206 * x * x * x + 1.52224 * x * x - 93.96192 * x + 3493.02631,
                0, 2400
        );
    }
}