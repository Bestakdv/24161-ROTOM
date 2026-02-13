package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LimelightHelper {

    private Limelight3A limelight;

    public void init(HardwareMap hwMap) {
        limelight = hwMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();
    }

    public LLResult getLatestResult() {
        return limelight.getLatestResult();
    }

    public void stop() {
        if (limelight != null) {
            limelight.stop();
        }
    }
}
