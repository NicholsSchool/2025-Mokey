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
import org.firstinspires.ftc.teamcode.math_utils.AutoUtil;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.components.IndicatorLight;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.concurrent.Callable;
import java.util.concurrent.TimeUnit;

@Autonomous(name = "Sample Score Auto", group = "Auto")
public class SampleAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        Drivetrain drivetrain = new Drivetrain(
                hardwareMap,
                new Pose2D(DistanceUnit.INCH, 24, 64, AngleUnit.DEGREES, 0),
                270, false, false
        );

        Elevator elevator = new Elevator(hardwareMap, false);

        Intake intake = new Intake(hardwareMap, false);

        drivetrain.leftLight.setColour(IndicatorLight.Colour.GREEN);
        drivetrain.rightLight.setColour(IndicatorLight.Colour.GREEN);

        AutoUtil.supplyOpModeActive(this::opModeIsActive);

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        telemetry.setMsTransmissionInterval(50);

        List<Callable<AutoUtil.AutoActionState>> actionSet = new ArrayList<>();
        List<Runnable> methodSet = new ArrayList<>();
        methodSet.add(drivetrain::update);
        methodSet.add(elevator::periodic);
        methodSet.add(() -> drivetrain.sendDashboardPacket(dashboard));

        waitForStart();

        intake.setWristSetpoint(IntakeConstants.WRIST_HANDOFF);
        intake.setWristState(Intake.WRIST_STATE.GO_TO_POS);

        //go to basket and raise elevator
        elevator.setArmSetpoint(ElevatorConstants.ARM_BASKET);
        elevator.runArmGrabber(-0.5);
        actionSet.add( () -> drivetrain.driveToPose(new Pose2D(DistanceUnit.INCH, 55, 55, AngleUnit.DEGREES, 45), false) );
        actionSet.add( () -> elevator.setElevatorSetpoint(ElevatorConstants.SAMPLE_HIGH_BASKET));
        AutoUtil.runActionsConcurrent(actionSet, methodSet, TimeUnit.SECONDS, 6);
        actionSet.clear();
        drivetrain.stop();
        AutoUtil.runTimedLoop(methodSet, TimeUnit.SECONDS, 1);

        //push into basket
        actionSet.add( () -> drivetrain.driveToPose(new Pose2D(DistanceUnit.INCH, 60, 60, AngleUnit.DEGREES, 45), true) );
        AutoUtil.runActionsConcurrent(actionSet, methodSet, TimeUnit.SECONDS, 2);
        actionSet.clear();

        //outtake
        elevator.runArmGrabber(1);
        AutoUtil.runTimedLoop(methodSet, TimeUnit.SECONDS, 0.5);

        //pull away from basket
        actionSet.add( () -> drivetrain.driveToPose(new Pose2D(DistanceUnit.INCH, 55, 55, AngleUnit.DEGREES, 45), true) );
        AutoUtil.runActionsConcurrent(actionSet, methodSet, TimeUnit.SECONDS, 2);
        actionSet.clear();

        //retract elevator and go to samples
        actionSet.add( () -> elevator.setElevatorSetpoint(ElevatorConstants.WAYPOINT_ZERO));
        actionSet.add( () -> drivetrain.driveToPose( new Pose2D( DistanceUnit.INCH, -34.5, 40, AngleUnit.DEGREES, 180 ), false ) );
        AutoUtil.runActionsConcurrent(actionSet, methodSet, TimeUnit.SECONDS, 6);
        actionSet.clear();

        //align with sample row and lower intake
        actionSet.add( () -> drivetrain.driveToPose( new Pose2D( DistanceUnit.INCH, -34.5, 24, AngleUnit.DEGREES, 180 ), false ) );
        actionSet.add( () -> intake.setIntakeSetpoint(IntakeConstants.WAYPOINT_STOW));
        actionSet.add( () -> intake.setWristSetpoint(IntakeConstants.WRIST_INTAKE));
        AutoUtil.runActionsConcurrent(actionSet, methodSet, TimeUnit.SECONDS, 3);
        actionSet.clear();

        //start intaking and go into samples
        intake.runIntake(-1);
        actionSet.add( () -> drivetrain.driveToPose( new Pose2D( DistanceUnit.INCH, -40, 24, AngleUnit.DEGREES, 180 ), false ) );
        AutoUtil.runActionsConcurrent(actionSet, methodSet, TimeUnit.SECONDS, 3);
        intake.runIntake(0);
        actionSet.clear();

        //bring intake back in and raise elevator slightly
        actionSet.add( () -> elevator.setElevatorSetpoint(-200));
        actionSet.add( () -> intake.setIntakeSetpoint(IntakeConstants.WAYPOINT_RETRACT));
        actionSet.add( () -> elevator.setArmSetpoint(ElevatorConstants.ARM_HANDOFF));
        actionSet.add( () -> intake.setWristSetpoint(IntakeConstants.WRIST_HANDOFF));
        AutoUtil.runActionsConcurrent(actionSet, methodSet, TimeUnit.SECONDS, 1);
        actionSet.clear();

        //bring down elevator
        actionSet.add( () -> elevator.setElevatorSetpoint(ElevatorConstants.WAYPOINT_ZERO));
        AutoUtil.runActionsConcurrent(actionSet, methodSet, TimeUnit.SECONDS, 1);
        actionSet.clear();

        //handoff
        intake.runIntake(1);
        elevator.runArmGrabber(-1);
        AutoUtil.runTimedLoop(methodSet, TimeUnit.SECONDS, 2);
        intake.runIntake(0);
        elevator.runArmGrabber(-0.2);

        //prepare to return to basket
        actionSet.add( () -> drivetrain.driveToPose( new Pose2D( DistanceUnit.INCH, -34.5, 40, AngleUnit.DEGREES, 90 ), false ) );
        AutoUtil.runActionsConcurrent(actionSet, methodSet, TimeUnit.SECONDS, 6);
        actionSet.clear();

        //return to basket and raise elevator
        actionSet.add( () -> drivetrain.driveToPose( new Pose2D( DistanceUnit.INCH, 55, 55, AngleUnit.DEGREES, 45 ), false ) );
        actionSet.add( () -> elevator.setElevatorSetpoint(ElevatorConstants.SAMPLE_HIGH_BASKET));
        actionSet.add( () -> elevator.setArmSetpoint(ElevatorConstants.ARM_BASKET));
        AutoUtil.runActionsConcurrent(actionSet, methodSet, TimeUnit.SECONDS, 6);
        actionSet.clear();
        AutoUtil.runTimedLoop(methodSet, TimeUnit.SECONDS, 1);

        //outtake
        elevator.runArmGrabber(1);
        AutoUtil.runTimedLoop(methodSet, TimeUnit.SECONDS, 0.5);

        //retract elevator
        actionSet.add( () -> drivetrain.driveToPose( new Pose2D( DistanceUnit.INCH, 52, 52, AngleUnit.DEGREES, 90 ), false ) );
        actionSet.add( () -> elevator.setElevatorSetpoint(ElevatorConstants.WAYPOINT_ZERO));
        actionSet.add( () -> elevator.setArmSetpoint(ElevatorConstants.ARM_STOW));
        AutoUtil.runActionsConcurrent(actionSet, methodSet, TimeUnit.SECONDS, 6);
        actionSet.clear();

        //park
        actionSet.add( () -> drivetrain.driveToPose(new Pose2D(DistanceUnit.INCH, 55, 55, AngleUnit.DEGREES, 90), true) );
        AutoUtil.runActionsConcurrent(actionSet, methodSet, TimeUnit.SECONDS, 2);
        actionSet.clear();
        drivetrain.stop();

        elevator.setArmSetpoint(ElevatorConstants.ARM_STOW);
        AutoUtil.runTimedLoop(methodSet, TimeUnit.SECONDS, 1);
    }
}
