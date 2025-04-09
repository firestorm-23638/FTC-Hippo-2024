package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.roadrunner.GoBildaPinpointDriver;

@TeleOp
public class TestPinpoint extends CommandOpMode {
    private GoBildaPinpointDriver driver;

    @Override
    public void initialize() {
        driver = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        driver.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);

        driver.resetPosAndIMU();

        waitForStart();
        schedule(new RunCommand(telemetry::update));

        schedule(new RunCommand(() -> {
            driver.update();

            telemetry.addData("par ticks", driver.getEncoderX());
            telemetry.addData("perp ticks", driver.getEncoderY());
        }));
    }
}
