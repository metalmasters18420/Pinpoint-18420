package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;

//import org.firstinspires.ftc.teamcode.AutoHardware.AutoArm;
import org.firstinspires.ftc.teamcode.MecanumDrive;


@Config
@Disabled
@Autonomous(name="Red Obs Just Park", group = "Auto")
public class RedObserveAuto extends LinearOpMode {
    @Override
    public void runOpMode() {
        Pose2d RedObservePose = new Pose2d(12, -60, Math.toRadians(90));
       // Pose2d RedBucketPose = new Pose2d(-18, -60, Math.toRadians(90));
       // Pose2d BlueBucketPose = new Pose2d(12, 60, Math.toRadians(-90));
        //Pose2d BlueObservePose = new Pose2d(-18, 60, Math.toRadians(-90));

        Vector2d DEEP_END_POINT_RED = new Vector2d(60,-60);
        Vector2d SHALLOW_END_POINT_RED = new Vector2d(48,-60);
        Vector2d DEEP_END_POINT_BLUE = new Vector2d(-60, 60);
        Vector2d SHALLOW_END_POINT_BLUE = new Vector2d(-48, 60);

        MecanumDrive drive = new MecanumDrive(hardwareMap, RedObservePose);
        //AutoArm arm = new AutoArm(hardwareMap);
        //AutoArm claw = new AutoArm(hardwareMap);
        //Claw claw = new Claw(hardwareMap);
        //Lift lift = new Lift(hardwareMap);



        TrajectoryActionBuilder tab1 = drive.actionBuilder(RedObservePose)
                .splineTo(new Vector2d(-11,-40),Math.toRadians(90))
                .turn(Math.toRadians(180))
                .lineToY(-32.5)
                .waitSeconds(2)
                .lineToY(-35)
                .setTangent(Math.toRadians(-10))
                .splineTo(new Vector2d(38,-10),Math.toRadians(100))
                .turnTo(Math.toRadians(0))
                .setTangent(Math.toRadians(0))
                .lineToX(48)
                .turnTo(Math.toRadians(90))
                .setTangent(Math.toRadians(90))
                .lineToY(SHALLOW_END_POINT_RED.y)
                .setTangent(Math.toRadians(90))
                .lineToY(-10)
                .setTangent(Math.toRadians(0))
                .lineToX(58)
                .setTangent(Math.toRadians(90))
                .lineToY(SHALLOW_END_POINT_RED.y)
                .setTangent(Math.toRadians(90))
                .lineToY(-10)
                .setTangent(Math.toRadians(0))
                .lineToX(62)
                .setTangent(Math.toRadians(90))
                .lineToY(SHALLOW_END_POINT_RED.y);
        ////Lift lift = new Lift(hardwareMap); */

        Action trajectoryActionCloseOut = tab1.fresh()
                .strafeTo(new Vector2d(48, 12))
                .build();

        Action tab11;
        tab11 = tab1.build();
       /* Actions.runBlocking(
                new SequentialAction(
                        tab11,
                        trajectoryActionCloseOut
                )
        ); */
        Actions.runBlocking(
                new SequentialAction(
//                        arm.rotateUp(),
//                        new SleepAction(1),
//                        claw.clawDrop(),
//                        new SleepAction(1),
//                        arm.rotateDown(),
//                        new SleepAction(1),
//                        claw.armRestore()

                ));


    }
}


