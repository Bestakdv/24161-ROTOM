package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

@TeleOp
@Configurable
public class AutoLockTestLL extends OpMode {
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor rightBack;

    private final LimelightHelper limelightHelper = new LimelightHelper();

    // PD stuff
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

        // --- Initialize Motors ---
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack   = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack  = hardwareMap.get(DcMotor.class, "rightBack");

        // --- Motor Directions ---
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);

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
        strafe = gamepad1.left_stick_x * 1.1;
        rotate = gamepad1.right_stick_x;

        LLResult result = limelightHelper.getLatestResult();

        Double targetTx = null;

        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            for (LLResultTypes.FiducialResult tag : fiducials) {
                if (tag.getFiducialId() == 20) {
                    Pose3D tagPose = tag.getCameraPoseTargetSpace();

                    double x = tagPose.getPosition().x;
                    double z = tagPose.getPosition().z;

                    targetTx = Math.toDegrees(Math.atan2(x, z));
                    break;
                }
            }
        }

        if(gamepad1.left_trigger > 0.3){
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
                // Tag 20 not found
                lastTime = getRuntime();
                lastError = 0;
            }
        } else {
            lastError = 0;
            lastTime = getRuntime();
        }

        // 3. Mecanum Drive Math
        double denominator = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(rotate), 1);

        double leftFrontPower  = (forward + strafe + rotate) / denominator;
        double rightFrontPower = (forward - strafe - rotate) / denominator;
        double leftBackPower   = (forward - strafe + rotate) / denominator;
        double rightBackPower  = (forward + strafe - rotate) / denominator;

        leftFront.setPower(leftFrontPower);
        rightFront.setPower(rightFrontPower);
        leftBack.setPower(leftBackPower);
        rightBack.setPower(rightBackPower);

        telemetry.addData("ID 20 Found", targetTx != null);
        telemetry.addData("Calculated tx", targetTx);
        telemetry.update();
    }

    @Override
    public void stop() {
        limelightHelper.stop();
    }
}
