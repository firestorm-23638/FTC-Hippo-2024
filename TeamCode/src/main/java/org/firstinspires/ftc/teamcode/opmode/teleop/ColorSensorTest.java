package org.firstinspires.ftc.teamcode.opmode.teleop;


import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.TwoWayColorSensor;

@TeleOp
public class ColorSensorTest extends CommandOpMode {
    private TwoWayColorSensor sensor;
    public Intake.color[] colorSamples = new Intake.color[100];
    public short colorSampleAmt = 0;
    public Intake.color finalSampleColor;

    @Override
    public void initialize() {
        sensor = new TwoWayColorSensor(hardwareMap, telemetry);

        register(sensor);

        waitForStart();
        schedule(new RunCommand(telemetry::update));

        // Put game start code here. i.e home everything
        schedule(new RunCommand(() -> {
            sensor.update();
            telemetry.addData("Color", sensor.getColor());
        }));
    }
}