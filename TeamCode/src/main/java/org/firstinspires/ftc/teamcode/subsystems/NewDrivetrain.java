package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveKinematics;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

public class NewDrivetrain extends SubsystemBase {
    private static int frontLeft = 0;
    private static int frontRight = 0;
    private static int backLeft = 0;
    private static int backRight = 0;

    private MecanumDrive mecanumDrive;
    public NewDrivetrain(HardwareMap hmap, Pose2d pose) {
        // get motors for drivetrain

        mecanumDrive = new MecanumDrive(hmap,pose);
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
    }

    public void driveArcade(double forwardSpeed, double strafeSpeed, double turnSpeed) {
        this.fieldCentricDrive(forwardSpeed, strafeSpeed, turnSpeed, 0.0);
    }

    public void driveFieldCentric(double forwardSpeed, double strafeSpeed, double turnSpeed) {
        Rotation2d gyroAngle = this.mecanumDrive.pose.heading;
        this.fieldCentricDrive(forwardSpeed, strafeSpeed, turnSpeed, gyroAngle.toDouble());
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
    }

    public TrajectoryActionBuilder getTrajectoryBuilder(Pose2d initalPose) {
        return this.mecanumDrive.actionBuilder(initalPose);
    }
}