package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.math_utils.NewRobotPose;

@TeleOp(name = "Pose Testing", group = "Dev")
public class PoseTesting extends OpMode {

    private NewRobotPose pose;


    @Override
    public void init() {

        pose = new NewRobotPose(hardwareMap, new Pose2D(DistanceUnit.METER, 0, 0, AngleUnit.DEGREES, 270), false);

    }

    @Override
    public void loop() {

        pose.update();

        telemetry.addData("OTOS Heading", pose.otos.getHeading());
        telemetry.addData("OTOS Position", pose.otos.getPosition().toString());

        telemetry.addData("Initial Pose", pose.initialPose.toString());
        telemetry.addData("Robot Pose", pose.debugTransform().toString());

        telemetry.update();

    }
}
