package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class Limelight extends SubsystemBase {
    private final Telemetry telemetry;
    private Pose3D currPose;

    private final Limelight3A limelight;

    public Limelight(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        this.limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();
    }

    @Override
    public void periodic() {
        telemetry.addData("Limelight X", currPose.getPosition().x);
        telemetry.addData("Limelight Y",  currPose.getPosition().y);
        telemetry.addData("Limelight Angle", currPose.getOrientation().getYaw(AngleUnit.DEGREES));
    }

    public void updatePosition() {
        limelight.pipelineSwitch(0);
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            currPose = result.getBotpose();
        }
    }

    public Pose2d getCurrentPose() {
        if (currPose == null) {
            return null;
        }
        return new Pose2d(currPose.getPosition().x, currPose.getPosition().y, currPose.getOrientation().getYaw(AngleUnit.DEGREES));
    }
}