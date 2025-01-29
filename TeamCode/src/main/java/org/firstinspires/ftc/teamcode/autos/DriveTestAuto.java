package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.constants.ElevatorConstants;
import org.firstinspires.ftc.teamcode.math_utils.AutoUtil;
import org.firstinspires.ftc.teamcode.math_utils.Vector;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.Callable;
import java.util.concurrent.TimeUnit;

@Autonomous(name = "Drive Test Auto", group = "Dev")
public class DriveTestAuto extends LinearOpMode {

    Drivetrain drivetrain;

    @Override
    public void runOpMode() {

        drivetrain = new Drivetrain(hardwareMap, new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0), 270, false);

        AutoUtil.supplyOpModeActive(this::opModeIsActive);

        drivetrain.resetElevatorEncoder();

        waitForStart();

        List<Callable<AutoUtil.AutoActionState>> actionSet = new ArrayList<>();
        List<Runnable> timedActionSet = new ArrayList<>();

        List<Runnable> methodSet1 = new ArrayList<>();
        methodSet1.add(drivetrain::update);
        methodSet1.add(() -> telemetry.addData("POSE", drivetrain.getPose().toString()));
        methodSet1.add(() -> telemetry.addLine(AutoUtil.getLoopStatesReadout()));
        methodSet1.add(() -> telemetry.update());

        actionSet.add( () -> drivetrain.driveToPose( new Pose2D( DistanceUnit.INCH, 0, -20, AngleUnit.DEGREES, 270 ), false ) );
        AutoUtil.runActionsConcurrent( actionSet, methodSet1, TimeUnit.SECONDS, 5);

        actionSet.clear();
        actionSet.add( () -> drivetrain.driveToPose( new Pose2D( DistanceUnit.INCH, 20, -20, AngleUnit.DEGREES, 0 ), false ) );
        AutoUtil.runActionsConcurrent( actionSet, methodSet1, TimeUnit.SECONDS, 5);

        actionSet.clear();
        actionSet.add( () -> drivetrain.driveToPose( new Pose2D( DistanceUnit.INCH, 20, 0, AngleUnit.DEGREES, 90 ), false ) );
        AutoUtil.runActionsConcurrent( actionSet, methodSet1, TimeUnit.SECONDS, 5);

        actionSet.clear();
        actionSet.add( () -> drivetrain.driveToPose( new Pose2D( DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 180 ), false ) );
        AutoUtil.runActionsConcurrent( actionSet, methodSet1, TimeUnit.SECONDS, 5);

    }
}
