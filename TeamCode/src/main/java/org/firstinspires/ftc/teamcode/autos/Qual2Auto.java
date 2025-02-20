package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.constants.ElevatorConstants;
import org.firstinspires.ftc.teamcode.math_utils.AutoUtil;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.components.IndicatorLight;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.Callable;
import java.util.concurrent.TimeUnit;

@Autonomous(name = "Qual 2 Auto", group = "Auto")
public class Qual2Auto extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        Drivetrain drivetrain = new Drivetrain(
                hardwareMap,
                new Pose2D(DistanceUnit.INCH,48, 56, AngleUnit.RADIANS, 0),
                270, true, false
        );

        Elevator elevator = new Elevator(hardwareMap, false);

        Intake intake = new Intake(hardwareMap, false);

        AutoUtil.supplyOpModeActive(this::opModeIsActive);

        intake.resetSlideEncoder();
        elevator.resetSlideEncoder();

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        telemetry.setMsTransmissionInterval(50);

        drivetrain.leftLight.setColour(IndicatorLight.Colour.GREEN);
        drivetrain.rightLight.setColour(IndicatorLight.Colour.GREEN);

        waitForStart();

        drivetrain.leftLight.setColour(IndicatorLight.Colour.YELLOW);
        drivetrain.rightLight.setColour(IndicatorLight.Colour.YELLOW);

        List<Callable<AutoUtil.AutoActionState>> actionSet = new ArrayList<>();
        List<Runnable> timedActionSet = new ArrayList<>();

        List<Runnable> methodSet1 = new ArrayList<>();
        methodSet1.add(drivetrain::update);
        methodSet1.add(elevator::periodic);
        //methodSet1.add(() -> intake.manualControl(-0.3));
        methodSet1.add(intake::periodic);
        methodSet1.add(() -> telemetry.addData("POSE", drivetrain.getPose().toString()));
        methodSet1.add(() -> telemetry.addData("ELEVATOR", elevator.getEncoderPosition()));
        methodSet1.add(() -> telemetry.addData("Loop Time", AutoUtil.getLoopTime()));
        methodSet1.add(() -> drivetrain.sendDashboardPacket(dashboard));
        methodSet1.add(() -> telemetry.addLine(AutoUtil.getLoopStatesReadout()));
        methodSet1.add(() -> telemetry.update());

        actionSet.add(() -> drivetrain.driveToPose(new Pose2D(DistanceUnit.INCH, 48, 0, AngleUnit.DEGREES, 0), true));
        AutoUtil.runActionsConcurrent(actionSet, methodSet1, TimeUnit.SECONDS, 5);

        actionSet.clear();
        actionSet.add(() -> drivetrain.driveToPose(new Pose2D(DistanceUnit.INCH, 24, 0, AngleUnit.DEGREES, 0), true));
        actionSet.add( () -> elevator.setElevatorSetpoint(ElevatorConstants.SPECIMEN_PULL));
        AutoUtil.runActionsConcurrent(actionSet, methodSet1, TimeUnit.SECONDS, 30);

    }
}
