package subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;


/*
Drivetrain: The drivetrain subsystem for the robot.
*/
public class Drivetrain extends SubsystemBase {
    public Follower follower;
    private Telemetry telemetry;
    private HardwareMap hmap;
    private Pose currentPose;

    public double forwardSpeedlimit = 1;
    public double strafeSpeedlimit = 1;
    public double rotSpeedLimit = 1;

    public Drivetrain(HardwareMap hmap, Pose pose, Telemetry telemetry) {
        this.currentPose = pose;
        this.telemetry = telemetry;
        this.hmap = hmap;

        follower = new Follower(hmap, FConstants.class, LConstants.class);
        hmap.get(DcMotorEx.class, "frontLeft").setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hmap.get(DcMotorEx.class, "frontRight").setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hmap.get(DcMotorEx.class, "backLeft").setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hmap.get(DcMotorEx.class, "backRight").setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        follower.setStartingPose(currentPose);
    }

    public void initialize() {

    }

    @Override
    public void periodic() {
        telemetry.addData("Robot X", this.follower.getPose().getX());
        telemetry.addData("Robot Y", this.follower.getPose().getY());
        telemetry.addData("Robot Heading", Math.toDegrees(this.follower.getPose().getHeading()));

        this.follower.update();
    }

    public void setToBrakeMode() {
        hmap.get(DcMotorEx.class, "frontLeft").setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hmap.get(DcMotorEx.class, "frontRight").setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hmap.get(DcMotorEx.class, "backLeft").setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hmap.get(DcMotorEx.class, "backRight").setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setRawSpeeds(double forwardSpeed, double strafeSpeed, double turnSpeed) {
        this.follower.setTeleOpMovementVectors(forwardSpeed*forwardSpeedlimit, strafeSpeed*strafeSpeedlimit, turnSpeed*rotSpeedLimit);
    }

    public Pose getCurrentPose() {
        return this.follower.getPose();
    }

    public void switchToTeleop() {
        this.follower.startTeleopDrive();
    }

    public void driveFieldCentric(double forwardSpeed, double strafeSpeed, double turnSpeed, boolean isFieldCentric) {
        this.follower.setTeleOpMovementVectors(forwardSpeed*forwardSpeedlimit, strafeSpeed*strafeSpeedlimit, turnSpeed*rotSpeedLimit, isFieldCentric);
    }

    public PathBuilder getBuilder() {
        return follower.pathBuilder();
    }
}