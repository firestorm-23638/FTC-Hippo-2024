package opmode.auto;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import Constants.Constants;
import commands.PathChainCommand;
import subsystems.Drivetrain;

@Autonomous
public class TestPaths extends CommandOpMode {
    private Drivetrain drive;

    @Override
    public void initialize() {
        Constants.IS_RED = false;

        Pose home = new Pose(9.323, 111.20383935829119, Math.toRadians(0));
        drive = new Drivetrain(hardwareMap, home, telemetry);

        drive.forwardSpeedlimit = 1;
        drive.strafeSpeedlimit = 1;
        drive.rotSpeedLimit = 1;

        PathChain startToBasket = Actions.startToBasket(home);

        PathChain basketToFirstSample = Actions.basketToFirstSample();
        PathChain basketToSecondSample = Actions.basketToSecondSample();
        PathChain basketToThirdSample = Actions.basketToThirdSample();
        PathChain firstSampleToBasket = Actions.firstSampleToBasket();
        PathChain secondSampleToBasket = Actions.secondSampleToBasket();
        PathChain thirdSampleToBasket = Actions.thirdSampleToBasket();
        PathChain basketToSubmersible = Actions.basketToSubmersible2(drive);
        PathChain submersibleToBasket = Actions.submersibleToBasket(drive);
        PathChain basketToSubmersible2 = Actions.basketToSubmersible(drive);
        PathChain submersible2ToBasket = Actions.submersible2ToBasket(drive);

        register(drive);
        schedule(new RunCommand(telemetry::update));
        waitForStart();

        schedule(new SequentialCommandGroup(
                new PathChainCommand(startToBasket, drive),

                new PathChainCommand(basketToFirstSample, drive),
                new PathChainCommand(firstSampleToBasket, drive),

                new PathChainCommand(basketToSecondSample, drive),
                new PathChainCommand(secondSampleToBasket, drive),
//
                new PathChainCommand(basketToThirdSample, drive),
                new PathChainCommand(thirdSampleToBasket, drive)
        ));
    }
}
