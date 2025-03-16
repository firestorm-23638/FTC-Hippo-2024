package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;

import java.util.List;

public class Limelight extends SubsystemBase {
    private final Telemetry telemetry;
    private LLResult currResult;
    public short currentPipeline = 0;

    private final Limelight3A limelight;

    public Limelight(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        this.limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(2);
        limelight.setPollRateHz(100);
        limelight.start();
    }

    @Override
    public void periodic() {
        lookForSamples(); // for now
//        if (currResult != null) {
//            telemetry.addData("Limelight X", currResult.getTx() * 39.26);
//            telemetry.addData("Limelight Y",  currResult.getTy() * 39.26);
//            telemetry.addData("Limelight Bot pose", currResult.getBotpose().toString());
//        }

    }

    public void updatePosition() {
        if (currentPipeline != Constants.LIMELIGHT_APRILTAG_PIPELINE) {
            limelight.pipelineSwitch(Constants.LIMELIGHT_APRILTAG_PIPELINE);
        }
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            currResult = result;
        }
    }

    public void toApriltags() {
        limelight.pipelineSwitch(1);
    }

    public void toYellowSample() {
        limelight.pipelineSwitch(2);
    }

    public void toBlueSample() {
        limelight.pipelineSwitch(0);
    }

    public List<LLResultTypes.ColorResult> lookForSamples() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            List<LLResultTypes.ColorResult> colorResults = result.getColorResults();
            for (LLResultTypes.ColorResult cr : colorResults) {
                telemetry.addData("Color", "X: %.2f, Y: %.2f", cr.getTargetXDegrees(), cr.getTargetYDegrees());
            }
            return colorResults;
        }
        return null;
    }

    /*public Pose2d getCurrentPose() {
        if (currPose == null) {
            return null;
        }
        return new Pose2d(currPose.getPosition().x, currPose.getPosition().y, currPose.getOrientation().getYaw(AngleUnit.DEGREES));
    }*/
}