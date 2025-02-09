package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.constants.ElevatorConstants;
import org.firstinspires.ftc.teamcode.constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.constants.TestAutoConstants;
import org.firstinspires.ftc.teamcode.math_utils.AutoUtil;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.components.IndicatorLight;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.Callable;
import java.util.concurrent.TimeUnit;

@Autonomous(name = "Systems Test", group = "Dev")
public class TestAuto extends LinearOpMode implements TestAutoConstants {

    Drivetrain drivetrain;
    Elevator elevator;
    Intake intake;
    FtcDashboard dashboard;

    @Override
    public void runOpMode() {

        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        telemetry.setMsTransmissionInterval(50);

        drivetrain = new Drivetrain(hardwareMap, new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0), 270, false, false);
        elevator = new Elevator(hardwareMap, false);
        intake = new Intake(hardwareMap, false);

        AutoUtil.supplyOpModeActive(this::opModeIsActive);

        List<Callable<AutoUtil.AutoActionState>> actionSet = new ArrayList<>();

        List<Runnable> methodSet1 = new ArrayList<>();
        methodSet1.add(drivetrain::update);
        methodSet1.add(intake::periodic);
        methodSet1.add(elevator::periodic);
        methodSet1.add(() -> telemetry.addData("POSE", drivetrain.getPose().toString()));
        methodSet1.add(() -> telemetry.addLine(AutoUtil.getLoopStatesReadout()));
        methodSet1.add(() -> telemetry.addLine(elevator.getTelemetry()));
        methodSet1.add(() -> telemetry.addLine(intake.getTelemetry()));
        methodSet1.add(() -> drivetrain.sendDashboardPacket(dashboard));
        methodSet1.add(telemetry::update);

        drivetrain.leftLight.setColour(IndicatorLight.Colour.GREEN);
        drivetrain.rightLight.setColour(IndicatorLight.Colour.GREEN);

        waitForStart();

        if (RUN_ELEVATOR_TESTS) actionSet.add( () -> elevator.setElevatorSetpoint(ElevatorConstants.SPECIMEN_READY) );
        if (RUN_INTAKE_TESTS) actionSet.add( () -> intake.setIntakeSetpoint(IntakeConstants.WAYPOINT_EXTEND) );
        AutoUtil.runActionsConcurrent(actionSet, methodSet1, TimeUnit.SECONDS, 5);

        actionSet.clear();
        if (RUN_ELEVATOR_TESTS) actionSet.add( () -> elevator.setElevatorSetpoint(ElevatorConstants.WAYPOINT_ZERO) );
        if (RUN_INTAKE_TESTS) actionSet.add( () -> intake.setIntakeSetpoint(IntakeConstants.WAYPOINT_RETRACT) );
        AutoUtil.runActionsConcurrent(actionSet, methodSet1, TimeUnit.SECONDS, 5);

        if (!RUN_DRIVETRAIN_TESTS) return;

        actionSet.clear();
        actionSet.add( () -> drivetrain.driveToPose( new Pose2D( DistanceUnit.INCH, 0, -48, AngleUnit.DEGREES, 0 ), false ) );
        AutoUtil.runActionsConcurrent( actionSet, methodSet1, TimeUnit.SECONDS, 5);

        actionSet.clear();
        actionSet.add( () -> drivetrain.driveToPose( new Pose2D( DistanceUnit.INCH, 24, -48, AngleUnit.DEGREES, 0 ), false ) );
        AutoUtil.runActionsConcurrent( actionSet, methodSet1, TimeUnit.SECONDS, 5);

    }
}
