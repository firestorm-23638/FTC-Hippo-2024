package opmode.teleop;


import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import subsystems.TwoWayColorSensor;

@TeleOp
public class ColorSensorTest extends CommandOpMode {
    private TwoWayColorSensor sensor;

    @Override
    public void initialize() {
        sensor = new TwoWayColorSensor(hardwareMap, telemetry);

        register(sensor);

        waitForStart();
        schedule(new RunCommand(telemetry::update));

        // Put game start code here. i.e home everything
        schedule(new RunCommand(() -> {
            telemetry.addData("Is Red", sensor.isRed());
            telemetry.addData("Is Blue", sensor.isBlue());
            telemetry.addData("Is Yellow", sensor.isYellow());
            telemetry.addData("Is None", sensor.isNone());
        }));
    }
}