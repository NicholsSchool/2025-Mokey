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
                new Pose2D(DistanceUnit.INCH,-16, 61, AngleUnit.RADIANS, 0),
                270, false
        );

        Elevator elevator = new Elevator(hardwareMap, false);

        Intake intake = new Intake(hardwareMap, false);

        AutoUtil.supplyOpModeActive(this::opModeIsActive);

        intake.resetSlideEncoder();
        elevator.resetSlideEncoder();

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        telemetry.setMsTransmissionInterval(50);

        waitForStart();

        List<Callable<AutoUtil.AutoActionState>> actionSet = new ArrayList<>();
        List<Runnable> timedActionSet = new ArrayList<>();

        List<Runnable> methodSet1 = new ArrayList<>();
        methodSet1.add(drivetrain::update);
        methodSet1.add(elevator::periodic);
        //methodSet1.add(() -> intake.manualControl(-0.3));
        methodSet1.add(intake::periodic);
        methodSet1.add(() -> telemetry.addData("POSE", drivetrain.getPose().toString()));
        methodSet1.add(() -> telemetry.addData("ELEVATOR", drivetrain.getElevatorPosition()));
        methodSet1.add(() -> drivetrain.sendDashboardPacket(dashboard));
        methodSet1.add(() -> telemetry.addLine(AutoUtil.getLoopStatesReadout()));
        methodSet1.add(() -> telemetry.update());

        elevator.setArmSetpoint(ElevatorConstants.ARM_HANDOFF);

        //Drive to chamber and set elevator to ready position
        actionSet.add( () -> drivetrain.driveToPose(new Pose2D(DistanceUnit.INCH, 5.5, 35, AngleUnit.RADIANS, 0), true) );
        actionSet.add( () -> elevator.setElevatorSetpoint(ElevatorConstants.SPECIMEN_READY ) );
        AutoUtil.runActionsConcurrent(actionSet, methodSet1, TimeUnit.SECONDS, 4);

        //Push into submersible quickly
        timedActionSet.add(() -> drivetrain.drive(new Vector(0.0, 0.5), 0.0, false));
        AutoUtil.runTimedLoop(timedActionSet, TimeUnit.SECONDS, 0.05);
        timedActionSet.clear();

        //Pull down specimen onto chamber
        actionSet.clear();
        actionSet.add( () -> elevator.setElevatorSetpoint(ElevatorConstants.SPECIMEN_PULL ) );
        AutoUtil.runActionsConcurrent( actionSet, methodSet1, TimeUnit.SECONDS, 1 );

        //Pull off of specimen
        actionSet.clear();
        actionSet.add( () -> drivetrain.driveToPose( new Pose2D( DistanceUnit.INCH, 5.5, 40, AngleUnit.DEGREES, 0 ), false ) );
        AutoUtil.runActionsConcurrent( actionSet, methodSet1, TimeUnit.SECONDS, 2 );

        //Move from chamber to sweep location
        actionSet.clear();
        actionSet.add( () -> drivetrain.driveToPose( new Pose2D( DistanceUnit.INCH, -35, 42, AngleUnit.DEGREES, 0 ), true ) );
        actionSet.add( () -> elevator.setElevatorSetpoint(ElevatorConstants.WAYPOINT_ZERO) );
        AutoUtil.runActionsConcurrent( actionSet, methodSet1, TimeUnit.SECONDS, 1.3);

        //Get behind the blocks
        actionSet.clear();
        actionSet.add( () -> drivetrain.driveToPose( new Pose2D( DistanceUnit.INCH, -34.5, 10, AngleUnit.DEGREES, 270 ), false ) );
        AutoUtil.runActionsConcurrent( actionSet, methodSet1, TimeUnit.SECONDS, 1 );

        //Strafe to behind block 1
        actionSet.clear();
        actionSet.add( () -> drivetrain.driveToPose( new Pose2D( DistanceUnit.INCH, -46, 10, AngleUnit.DEGREES, 270), true ) );
        AutoUtil.runActionsConcurrent( actionSet, methodSet1, TimeUnit.SECONDS, 1 );

        //Push block 1 to observation
        actionSet.clear();
        actionSet.add( () -> drivetrain.driveToPose( new Pose2D( DistanceUnit.INCH, -46, 55, AngleUnit.DEGREES, 270), false ) );
        AutoUtil.runActionsConcurrent( actionSet, methodSet1, TimeUnit.SECONDS, 3 );

        // Return to behind blocks
        actionSet.clear();
        actionSet.add( () -> drivetrain.driveToPose( new Pose2D( DistanceUnit.INCH, -34.5, 10, AngleUnit.DEGREES, 270), false ) );
        AutoUtil.runActionsConcurrent( actionSet, methodSet1, TimeUnit.SECONDS, 3 );

        //Strafe to block 2
        actionSet.clear();
        actionSet.add( () -> drivetrain.driveToPose( new Pose2D( DistanceUnit.INCH, -54, 10, AngleUnit.DEGREES, 270), true ) );
        AutoUtil.runActionsConcurrent( actionSet, methodSet1, TimeUnit.SECONDS, .6 );

        //Push block 2 to observation
        actionSet.clear();
        actionSet.add( () -> drivetrain.driveToPose( new Pose2D( DistanceUnit.INCH, -54, 55, AngleUnit.DEGREES, 270), false ) );
        AutoUtil.runActionsConcurrent( actionSet, methodSet1, TimeUnit.SECONDS, 3 );

        //Wait for HP
        actionSet.clear();
        actionSet.add( () -> drivetrain.driveToPose( new Pose2D( DistanceUnit.INCH, -38, 50, AngleUnit.DEGREES, 0), false ) );
        AutoUtil.runActionsConcurrent( actionSet, methodSet1, TimeUnit.SECONDS, 3 );

        //Get ready for wall pick 1
        actionSet.clear();
        actionSet.add( () -> drivetrain.driveToPose( new Pose2D( DistanceUnit.INCH, -38, 62, AngleUnit.DEGREES, 0), true ) );
        AutoUtil.runActionsConcurrent( actionSet, methodSet1, TimeUnit.SECONDS, 3 );
for (int i = 0; i < 4; i++) {
        //Slam into wall for pick 1
        timedActionSet.add(() -> drivetrain.drive(new Vector(0.0, -0.7), 0.0, false));
        AutoUtil.runTimedLoop(timedActionSet, TimeUnit.SECONDS, 0.05);
        timedActionSet.clear();

        //Pull specimen up and run to chamber
        actionSet.clear();
        int finalI = i;
        actionSet.add( () -> drivetrain.driveToPose( new Pose2D( DistanceUnit.METER, (-.07 - (finalI * .01)), 0.80, AngleUnit.DEGREES, 180), false ) );
        actionSet.add( () -> elevator.setElevatorSetpoint(ElevatorConstants.SPECIMEN_READY));
        AutoUtil.runActionsConcurrent( actionSet, methodSet1, TimeUnit.SECONDS, 5 );

        //Push into submersible quickly
        timedActionSet.add(() -> drivetrain.drive(new Vector(0.0, 0.5), 0.0, false));
        AutoUtil.runTimedLoop(timedActionSet, TimeUnit.SECONDS, 0.05);
        timedActionSet.clear();

        //Pull down specimen onto chamber
        actionSet.clear();
        actionSet.add( () -> elevator.setElevatorSetpoint(ElevatorConstants.SPECIMEN_PULL ) );
        AutoUtil.runActionsConcurrent( actionSet, methodSet1, TimeUnit.SECONDS, 1 );

        //Bring down elevator and go to HP
        actionSet.clear();
        actionSet.add( () -> elevator.setElevatorSetpoint(ElevatorConstants.WAYPOINT_ZERO));
        actionSet.add( () -> drivetrain.driveToPose( new Pose2D( DistanceUnit.METER, -0.92, 1.1, AngleUnit.DEGREES, 0), false ) );
        AutoUtil.runActionsConcurrent( actionSet, methodSet1, TimeUnit.SECONDS, 3 );

        //Get ready for wall pick
        actionSet.clear();
        actionSet.add( () -> drivetrain.driveToPose( new Pose2D( DistanceUnit.METER, -0.94, 1.4, AngleUnit.DEGREES, 0), false ) );
        AutoUtil.runActionsConcurrent( actionSet, methodSet1, TimeUnit.SECONDS, 1 );
        }

    }

    private static void stageDriveToPose()
    {

    }
}
