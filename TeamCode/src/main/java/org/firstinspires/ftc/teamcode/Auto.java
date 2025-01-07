package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.constants.ElevatorConstants;
import org.firstinspires.ftc.teamcode.math_utils.Vector;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;

import java.util.concurrent.TimeUnit;

/**
 * Testing Auto for Lerp
 */
@Autonomous(name="Contest Auto", group="Testing")
public class Auto extends LinearOpMode{

    @Override
    public void runOpMode() {
        Drivetrain drivetrain = new Drivetrain(hardwareMap, 0, 0, Math.PI, false);
        ElapsedTime time = new ElapsedTime();
        drivetrain.resetElevatorEncoder();
        Elevator elevator = new Elevator(hardwareMap, drivetrain::getElevatorPosition);

        waitForStart();
//        drivetrain.update();
//        while((Math.abs(24 - drivetrain.getPose(). x) > 2) && (Math.abs(24 - drivetrain.getPose().y) > 2)){
//            drivetrain.update();
//            double mag = Math.hypot(24 - drivetrain.getPose().x, 24 - drivetrain.getPose().y) * 3;
//            drivetrain.drive(new Vector((24 - drivetrain.getPose().x) / mag, (24 - drivetrain.getPose().y) / mag), 0, true);
////            drivetrain.drive(new Vector(0.1,0.1), 0, true);
//            telemetry.addData("x", drivetrain.getPose().x);
//            telemetry.addData("y", drivetrain.getPose().y);
//            telemetry.addData("driveX", (24 - drivetrain.getPose().x) / mag);
//            telemetry.addData("driveY", (24 - drivetrain.getPose().y) / mag);
//            telemetry.update();
//        }
        while(time.time(TimeUnit.SECONDS) < 15){
            drivetrain.drive(new Vector(0.0,0.0), 0, true);
        }
        drivetrain.setTargetHeading(drivetrain.getPose().angle);
        elevator.setSetpoint(ElevatorConstants.SPECIMEN_READY);
        while(time.time(TimeUnit.SECONDS) < 5.0){
            drivetrain.drive(new Vector(0.0,0.7), drivetrain.turnToAngle(), true);
            elevator.periodic();
            drivetrain.update();
        }
        time.reset();
        elevator.setSetpoint(ElevatorConstants.SPECIMEN_PULL);
        while(time.time(TimeUnit.SECONDS) < 1.5){
            elevator.periodic();
        }
        time.reset();
        elevator.setSetpoint(ElevatorConstants.WAYPOINT_ZERO);
        while(time.time(TimeUnit.SECONDS) < 3.0){
            drivetrain.drive(new Vector(0.8,-0.5), drivetrain.turnToAngle() , true);
            drivetrain.update();
            elevator.periodic();
        }
    }
}