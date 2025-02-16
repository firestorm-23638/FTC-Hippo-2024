package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;


/*
Drivetrain: The drivetrain subsystem for the robot.
*/
public class Drivetrain extends SubsystemBase {
    private static int frontLeft = 0;
    private static int frontRight = 1;
    private static int backLeft = 2;
    private static int backRight = 3;

    public Follower follower;
    private Telemetry telemetry;
    private double imuOffset;

    public double forwardSpeedlimit = 1;
    public double strafeSpeedlimit = 1;
    public double rotSpeedLimit = 1;

    public Drivetrain(HardwareMap hmap, Pose pose, Telemetry telemetry) {
        // get motors for drivetrain
        follower = new Follower(hmap);
        follower.setStartingPose(pose);
        this.telemetry = telemetry;
    }

    @Override
    public void periodic() {
        telemetry.addData("Robot X", this.follower.getPose().getX());
        telemetry.addData("Robot Y", this.follower.getPose().getY());
        telemetry.addData("Robot Heading", Math.toDegrees(this.follower.getPose().getHeading()));

        this.follower.update();
    }

    public void driveArcade(double forwardSpeed, double strafeSpeed, double turnSpeed) {
        this.follower.setTeleOpMovementVectors(forwardSpeed*forwardSpeedlimit, strafeSpeed*strafeSpeedlimit, turnSpeed*rotSpeedLimit);
    }

    public void setCurrentPose(Pose newPose) {
        this.follower.setPose(newPose);
    }

    public Pose getCurrentPose() {
        return this.follower.getPose();
    }

    public void driveFieldCentric(double forwardSpeed, double strafeSpeed, double turnSpeed, boolean isFieldCentric) {
        this.follower.setTeleOpMovementVectors(forwardSpeed*forwardSpeedlimit, strafeSpeed*strafeSpeedlimit, turnSpeed*rotSpeedLimit, isFieldCentric);
    }

    public PathBuilder getBuilder() {
        return follower.pathBuilder();
    }
}