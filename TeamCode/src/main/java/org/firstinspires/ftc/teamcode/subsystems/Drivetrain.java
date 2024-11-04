package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.arcrobotics.ftclib.hardware.RevIMU;
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

    private MecanumDrive mecanumDrive;
    private Telemetry telemetry;
    private double imuOffset;

    public Drivetrain(HardwareMap hmap, Pose2d pose, Telemetry telemetry) {
        // get motors for drivetrain
        mecanumDrive = new MecanumDrive(hmap,pose);
        this.telemetry = telemetry;
    }

    private double clipRange(double value) {
        return value <= -1.0 ? -1.0
                : value >= 1.0 ? 1.0
                : value;
    }

    /**
     * Normalize the wheel speeds
     */
    private void normalize(double[] wheelSpeeds, double magnitude) {
        double maxMagnitude = Math.abs(wheelSpeeds[0]);
        for (int i = 1; i < wheelSpeeds.length; i++) {
            double temp = Math.abs(wheelSpeeds[i]);
            if (maxMagnitude < temp) {
                maxMagnitude = temp;
            }
        }
        for (int i = 0; i < wheelSpeeds.length; i++) {
            wheelSpeeds[i] = (wheelSpeeds[i] / maxMagnitude) * magnitude;
        }

    }

    /**
     * Normalize the wheel speeds
     */
    private void normalize(double[] wheelSpeeds) {
        double maxMagnitude = Math.abs(wheelSpeeds[0]);
        for (int i = 1; i < wheelSpeeds.length; i++) {
            double temp = Math.abs(wheelSpeeds[i]);
            if (maxMagnitude < temp) {
                maxMagnitude = temp;
            }
        }
        if (maxMagnitude > 1) {
            for (int i = 0; i < wheelSpeeds.length; i++) {
                wheelSpeeds[i] = (wheelSpeeds[i] / maxMagnitude);
            }
        }
    }

    @Override
    public void periodic() {
        this.mecanumDrive.updatePoseEstimate();
        telemetry.addData("Robot X", this.mecanumDrive.pose.position.x);
        telemetry.addData("Robot Y", this.mecanumDrive.pose.position.y);
        telemetry.addData("Robot Heading", this.mecanumDrive.pose.heading.log());
    }

    public void driveArcade(double forwardSpeed, double strafeSpeed, double turnSpeed) {
        this.fieldCentricDrive(forwardSpeed, strafeSpeed, turnSpeed, 0.0);
    }

    public void setCurrentPose(Pose2d newPose) {
        this.mecanumDrive.pose = newPose;
    }

    public Pose2d getCurrentPose() {
        return this.mecanumDrive.pose;
    }

    public void driveFieldCentric(double forwardSpeed, double strafeSpeed, double turnSpeed) {

        this.fieldCentricDrive(forwardSpeed, strafeSpeed, turnSpeed, 0);//Math.toDegrees(mecanumDrive.pose.heading.log()) + 90);
    }

    private void fieldCentricDrive(double forwardSpeed, double strafeSpeed, double turnSpeed, double gyroAngle) {

        strafeSpeed = clipRange(strafeSpeed);
        forwardSpeed = clipRange(forwardSpeed);
        turnSpeed = clipRange(turnSpeed);
        Vector2d input = new Vector2d(strafeSpeed, forwardSpeed);
        input = input.rotateBy(-gyroAngle);

        double theta = input.angle();

        double[] wheelSpeeds = new double[4];
        wheelSpeeds[frontLeft] = Math.sin(theta + Math.PI / 4);
        wheelSpeeds[frontRight] = Math.sin(theta - Math.PI / 4);
        wheelSpeeds[backLeft] = Math.sin(theta - Math.PI / 4);
        wheelSpeeds[backRight] = Math.sin(theta + Math.PI / 4);

        normalize(wheelSpeeds, input.magnitude());

        wheelSpeeds[frontLeft] += turnSpeed;
        wheelSpeeds[frontRight] -= turnSpeed;
        wheelSpeeds[backLeft] += turnSpeed;
        wheelSpeeds[backRight] -= turnSpeed;

        normalize(wheelSpeeds);

        this.mecanumDrive.leftFront.setPower(wheelSpeeds[frontLeft]);
        this.mecanumDrive.rightFront.setPower(wheelSpeeds[frontRight]);
        this.mecanumDrive.leftBack.setPower(wheelSpeeds[backLeft]);
        this.mecanumDrive.rightBack.setPower(wheelSpeeds[backRight]);

        telemetry.addData("Front Left", wheelSpeeds[frontLeft]);
        telemetry.addData("Front Right", wheelSpeeds[frontRight]);
        telemetry.addData("Back Left", wheelSpeeds[backLeft]);
        telemetry.addData("Back Right", wheelSpeeds[backRight]);
    }

    public TrajectoryActionBuilder getTrajectoryBuilder(Pose2d initalPose) {
        return this.mecanumDrive.actionBuilder(initalPose);
    }

    public TrajectoryActionBuilder inverseGetTrajectoryBuilder(Pose2d initialPose) {
        return this.mecanumDrive.inverseActionBuilder(initialPose);
    }
}