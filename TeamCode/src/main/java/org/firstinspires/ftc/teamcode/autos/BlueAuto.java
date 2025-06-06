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
import org.firstinspires.ftc.teamcode.math_utils.Vector;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.components.IndicatorLight;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.Callable;
import java.util.concurrent.TimeUnit;

@Autonomous(name = "Blue Auto", group = "Auto")
public class BlueAuto extends LinearOpMode {
    @Override
    public void runOpMode() {

        Drivetrain drivetrain = new Drivetrain(
                hardwareMap,
                new Pose2D(DistanceUnit.INCH,-14, 64, AngleUnit.DEGREES, 0),
                270, false, false
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
        methodSet1.add(() -> drivetrain.leftLight.setColour(drivetrain.poseEstimator.isUsingLL() ? IndicatorLight.Colour.PURPLE : IndicatorLight.Colour.GREEN));
        methodSet1.add(() -> drivetrain.rightLight.setColour(drivetrain.poseEstimator.isUsingLL() ? IndicatorLight.Colour.PURPLE : IndicatorLight.Colour.GREEN));
        //methodSet1.add(() -> intake.manualControl(-0.3));
        methodSet1.add(intake::periodic);
        methodSet1.add(() -> telemetry.addData("POSE", drivetrain.getPose().toString()));
        methodSet1.add(() -> telemetry.addData("ELEVATOR", elevator.getEncoderPosition()));
        methodSet1.add(() -> telemetry.addData("Loop Time", AutoUtil.getLoopTime()));
        methodSet1.add(() -> drivetrain.sendDashboardPacket(dashboard));
        methodSet1.add(() -> telemetry.addLine(AutoUtil.getLoopStatesReadout()));
        methodSet1.add(() -> telemetry.update());

        elevator.setArmSetpoint(ElevatorConstants.ARM_HANDOFF);

        //Drive to chamber and set elevator to ready position
        actionSet.add( () -> drivetrain.driveToPose(new Pose2D(DistanceUnit.INCH, 5, 36, AngleUnit.RADIANS, 0), false) );
        actionSet.add( () -> elevator.setElevatorSetpoint(ElevatorConstants.SPECIMEN_READY ) );
        AutoUtil.runActionsConcurrent(actionSet, methodSet1, TimeUnit.SECONDS, 2);
        drivetrain.stop();

        //Push into submersible quickly
        drivetrain.drive(new Vector(0.0, 0.5), 0.0, true);
        AutoUtil.runTimedLoop(methodSet1, TimeUnit.SECONDS, 0.5);
        drivetrain.drive(new Vector(0.0, 0.0), 0.0, false);
        drivetrain.stop();

        //Pull down specimen onto chamber
        actionSet.clear();
        actionSet.add( () -> elevator.setElevatorSetpoint(ElevatorConstants.SPECIMEN_PULL ) );
        AutoUtil.runActionsConcurrent( actionSet, methodSet1, TimeUnit.SECONDS, 1 );
        drivetrain.stop();

        //Pull off of specimen
        actionSet.clear();
        actionSet.add( () -> drivetrain.driveToPose( new Pose2D( DistanceUnit.INCH, 0, 40, AngleUnit.DEGREES, 0 ), false ) );
        AutoUtil.runActionsConcurrent( actionSet, methodSet1, TimeUnit.SECONDS, 2 );
        drivetrain.stop();

        //Move from chamber to sweep location
        actionSet.clear();
        actionSet.add( () -> drivetrain.driveToPose( new Pose2D( DistanceUnit.INCH, -34.5, 40, AngleUnit.DEGREES, 0 ), false ) );
        actionSet.add( () -> elevator.setElevatorSetpoint(ElevatorConstants.WAYPOINT_ZERO) );
        AutoUtil.runActionsConcurrent( actionSet, methodSet1, TimeUnit.SECONDS, 1.3);
        drivetrain.stop();

        //Get behind the blocks
        actionSet.clear();
        actionSet.add( () -> drivetrain.driveToPose( new Pose2D( DistanceUnit.INCH, -34.5, 16, AngleUnit.DEGREES, 0 ), false ) );
        AutoUtil.runActionsConcurrent( actionSet, methodSet1, TimeUnit.SECONDS, 1 );
        drivetrain.stop();

        //Strafe to behind block 1
        actionSet.clear();
        actionSet.add( () -> drivetrain.driveToPose( new Pose2D( DistanceUnit.INCH, -44, 16, AngleUnit.DEGREES, 0), true ) );
        AutoUtil.runActionsConcurrent( actionSet, methodSet1, TimeUnit.SECONDS, 1 );
        drivetrain.stop();

        //Push block 1 to observation
        actionSet.clear();
        actionSet.add( () -> drivetrain.driveToPose( new Pose2D( DistanceUnit.INCH, -48, 52, AngleUnit.DEGREES, 0), false ) );
        AutoUtil.runActionsConcurrent( actionSet, methodSet1, TimeUnit.SECONDS, 3 );
        drivetrain.stop();

        //Get out of observation
        actionSet.clear();
        actionSet.add( () -> drivetrain.driveToPose( new Pose2D( DistanceUnit.INCH, -42, 36, AngleUnit.DEGREES, 0), false ) );
        AutoUtil.runActionsConcurrent( actionSet, methodSet1, TimeUnit.SECONDS, 3 );
        drivetrain.stop();

//        //Wait for HP
//        actionSet.clear();
//        actionSet.add( () -> drivetrain.driveToPose( new Pose2D( DistanceUnit.INCH, -42, 48, AngleUnit.DEGREES, 0), false ) );
//        AutoUtil.runActionsConcurrent( actionSet, methodSet1, TimeUnit.SECONDS, 3 );
//        drivetrain.stop();

        //Get ready for wall pick 1
        actionSet.clear();
        actionSet.add( () -> drivetrain.driveToPose( new Pose2D( DistanceUnit.INCH, -42, 56, AngleUnit.DEGREES, 0), true ) );
        AutoUtil.runActionsConcurrent( actionSet, methodSet1, TimeUnit.SECONDS, 1.5 );

for (int i = 0; i < 2; i++) {

        drivetrain.stop();
        AutoUtil.runTimedLoop(methodSet1, TimeUnit.SECONDS, 0.6);

        //Slam into wall for pick 1
        actionSet.clear();
        actionSet.add( () -> drivetrain.driveToPose( new Pose2D( DistanceUnit.INCH, -42, 64, AngleUnit.DEGREES, 0), true ) );
        AutoUtil.runActionsConcurrent( actionSet, methodSet1, TimeUnit.SECONDS, 0.7 );
        drivetrain.stop();

        //Pull specimen up and run to chamber
        actionSet.clear();
        int finalI = i;
        actionSet.add( () -> drivetrain.driveToPose( new Pose2D( DistanceUnit.INCH, ((finalI * -5)), 36, AngleUnit.DEGREES, 180), false ) );
        actionSet.add( () -> elevator.setElevatorSetpoint(ElevatorConstants.SPECIMEN_READY));
        AutoUtil.runActionsConcurrent( actionSet, methodSet1, TimeUnit.SECONDS, 3 );
        drivetrain.stop();

        //Push into submersible quickly
        drivetrain.drive(new Vector(0.0, 0.5), 0.0, true);
        AutoUtil.runTimedLoop(methodSet1, TimeUnit.SECONDS, 0.75);
        drivetrain.drive(new Vector(0.0, 0.0), 0.0, false);
        drivetrain.stop();

        //Pull down specimen onto chamber
        actionSet.clear();
        actionSet.add( () -> elevator.setElevatorSetpoint(ElevatorConstants.SPECIMEN_PULL ) );
        AutoUtil.runActionsConcurrent( actionSet, methodSet1, TimeUnit.SECONDS, 1 );
        drivetrain.stop();

        //pull away from chamber
        actionSet.clear();
        actionSet.add( () -> drivetrain.driveToPose( new Pose2D( DistanceUnit.INCH, -2.5, 40, AngleUnit.DEGREES, 0), false ) );
        AutoUtil.runActionsConcurrent(actionSet, methodSet1, TimeUnit.SECONDS, 1);

        //Bring down elevator and go to HP
        actionSet.clear();
        actionSet.add( () -> elevator.setElevatorSetpoint(ElevatorConstants.WAYPOINT_ZERO));
        actionSet.add( () -> drivetrain.driveToPose( new Pose2D( DistanceUnit.INCH, -42, 48, AngleUnit.DEGREES, 0), false ) );
        AutoUtil.runActionsConcurrent( actionSet, methodSet1, TimeUnit.SECONDS, 3 );
        drivetrain.stop();

        //Get ready for wall pick
        actionSet.clear();
        actionSet.add( () -> drivetrain.driveToPose( new Pose2D( DistanceUnit.INCH, -42, 56, AngleUnit.DEGREES, 0), false ) );
        AutoUtil.runActionsConcurrent( actionSet, methodSet1, TimeUnit.SECONDS, 1 );
        drivetrain.stop();
        }

actionSet.clear();
actionSet.add( () -> drivetrain.driveToPose( new Pose2D( DistanceUnit.INCH, -60, 60, AngleUnit.DEGREES, 0), false ) );
AutoUtil.runActionsConcurrent(actionSet, methodSet1, TimeUnit.SECONDS, 2);
drivetrain.stop();

    }
}
