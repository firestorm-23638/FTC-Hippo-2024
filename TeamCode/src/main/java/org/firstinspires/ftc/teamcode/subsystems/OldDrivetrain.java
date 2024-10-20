package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.GyroEx;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;

public class OldDrivetrain extends SubsystemBase {
    private final Motor frontLeft;
    private final Motor frontRight;
    private final Motor backLeft;
    private final Motor backRight;

    private final MecanumDrive mecanum;

    //private final RevIMU gyro;

    public OldDrivetrain(HardwareMap hardwareMap) {
        frontLeft = new Motor(hardwareMap, Constants.driveFrontLeftConfig);
        frontRight = new Motor(hardwareMap, Constants.driveFrontRightConfig);
        backLeft = new Motor(hardwareMap, Constants.driveBackLeftConfig);
        backRight = new Motor(hardwareMap, Constants.driveBackRightConfig);

        // I don't know why it's like this
        frontLeft.setInverted(false);
        frontRight.setInverted(true);
        backLeft.setInverted(false);
        backRight.setInverted(true);

        mecanum = new MecanumDrive(frontLeft, frontRight, backLeft, backRight);

        //gyro = new RevIMU(hardwareMap);

    };

    public void drive(double strafe, double forward, double turn) {
        mecanum.driveFieldCentric(strafe, forward, turn, 0);
    }
}
