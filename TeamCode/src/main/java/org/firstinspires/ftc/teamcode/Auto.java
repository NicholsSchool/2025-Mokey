package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.math_utils.Angles;
import org.firstinspires.ftc.teamcode.math_utils.LerpPathPlanning;
import org.firstinspires.ftc.teamcode.math_utils.Point;
import org.firstinspires.ftc.teamcode.math_utils.Vector;
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
        LerpPathPlanning spline = new LerpPathPlanning(drivetrain, new Point(24, 24), Math.PI);


        waitForStart();
        drivetrain.update();
        while(((24 - drivetrain.getPose().x) < 0.5) && ((24 - drivetrain.getPose().y) < 0.5)){
            drivetrain.update();
            double mag = Math.hypot(24 - drivetrain.getPose().x, 24 - drivetrain.getPose().y);
            drivetrain.drive(new Vector((24 - drivetrain.getPose().x) / mag, -(24 - drivetrain.getPose().y) / mag), 0, true);
            telemetry.addData("x", drivetrain.getPose().x);
            telemetry.addData("y", drivetrain.getPose().y);
            telemetry.update();
        }
//        boolean isFinished = false;
//        while(opModeIsActive() && !isFinished) {
//            isFinished = spline.spline(0.0, true, true);
//            telemetry.addData("x", drivetrain.getPose().x);
//            telemetry.addData("y", drivetrain.getPose().y);
//            telemetry.update();
//        }
    }
}