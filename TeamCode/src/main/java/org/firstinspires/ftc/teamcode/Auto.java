package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.math_utils.Angles;
import org.firstinspires.ftc.teamcode.math_utils.LerpPathPlanning;
import org.firstinspires.ftc.teamcode.math_utils.Point;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

/**
 * Testing Auto for Lerp
 */
@Autonomous(name="Lerp Testing", group="Testing")
public class Auto extends LinearOpMode{

    @Override
    public void runOpMode() {
        Drivetrain drivetrain = new Drivetrain(hardwareMap, 0, 0, Math.PI / 2, false);
        drivetrain.setFloat();
        LerpPathPlanning spline = new LerpPathPlanning(drivetrain, new Point(-24, -24), Math.PI / 2);


        waitForStart();

        boolean isFinished = false;
        while(opModeIsActive() && !isFinished) {
            isFinished = spline.spline(0.0, true, true);
        }
    }
}