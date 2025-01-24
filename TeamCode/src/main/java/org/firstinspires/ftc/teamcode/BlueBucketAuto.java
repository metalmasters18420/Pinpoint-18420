package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

// RR-specific imports

//import org.firstinspires.ftc.teamcode.AutoHardware.AutoArm;


@Config
@Autonomous(name="Blue Bucket Auto", group = "Auto")
public class BlueBucketAuto extends LinearOpMode {
    @Override
    public void runOpMode() {
        // Pose2d RedBucketPose = new Pose2d(12, -60, Math.toRadians(90));
        //Pose2d RedObservePose = new Pose2d(-18, -60, Math.toRadians(90));
        Pose2d BlueBucketPose = new Pose2d(12, 60, Math.toRadians(-90));
        Pose2d BlueObservePose = new Pose2d(-18, 60, Math.toRadians(-90));

        Vector2d DEEP_END_POINT_RED = new Vector2d(60, -60);
        Vector2d SHALLOW_END_POINT_RED = new Vector2d(48, -60);
        Vector2d DEEP_END_POINT_BLUE = new Vector2d(-60, 60);
        Vector2d SHALLOW_END_POINT_BLUE = new Vector2d(48, 50);

        //MecanumDrive drive = new MecanumDrive(hardwareMap, BlueObservePose);
        //AutoArm arm = new AutoArm(hardwareMap);
        //AutoArm claw = new AutoArm(hardwareMap);
        //Claw claw = new Claw(hardwareMap);
        //Lift lift = new Lift(hardwareMap);
//        MecanumDrive drive = new MecanumDrive(hardwareMap, BlueObservePose);
        hwRobot robot = new hwRobot();
        robot.init(hardwareMap);

        robot.drive.pose = new Pose2d(-40,-63,Math.toRadians(0));

        TrajectoryActionBuilder drivetobucket = robot.drive.actionBuilder(new Pose2d(-30, -60, Math.toRadians(0)))
                .strafeToSplineHeading(new Vector2d(-56, -55),Math.toRadians(45))
//                .waitSeconds(3)
                .endTrajectory();
        TrajectoryActionBuilder rightsample = drivetobucket.fresh()
                .strafeToSplineHeading(new Vector2d(-48, -30.7),Math.toRadians(90))
                //.turnTo(Math.toRadians(75))
                .waitSeconds(1)
                .endTrajectory();
        TrajectoryActionBuilder turntobinONE = rightsample.fresh()
                .strafeToSplineHeading(new Vector2d(-56, -55),Math.toRadians(45))
                //.turnTo(Math.toRadians(45))
                .waitSeconds(3)
                .endTrajectory();
        TrajectoryActionBuilder middlesample = turntobinONE.fresh()
                .turnTo(Math.toRadians(95))
                .waitSeconds(1)
                .endTrajectory();
        TrajectoryActionBuilder turntobinTWO = middlesample.fresh()
                .turnTo(Math.toRadians(45))
                .waitSeconds(3)
                .endTrajectory();
        TrajectoryActionBuilder leftsample = turntobinTWO.fresh()
                .turnTo(Math.toRadians(120))
                .waitSeconds(1)
                .endTrajectory();
        TrajectoryActionBuilder turntobinTHREE = leftsample.fresh()
                .turnTo(Math.toRadians(45))
                .waitSeconds(3)
                .endTrajectory();
        TrajectoryActionBuilder drivetosubmerse = turntobinTHREE.fresh()
                .strafeToSplineHeading(new Vector2d(-40, -10), Math.toRadians(180))
                .setTangent(Math.toRadians(0))
                .lineToX(-20)
                .endTrajectory();

        Action ToBucket = drivetobucket.build();

        Action ToRS = rightsample.build();

        Action Turn1 = turntobinONE.build();

        Action ToMS = middlesample.build();

        Action Turn2 = turntobinTWO.build();

        Action ToLS = leftsample.build();

        Action Turn3 = turntobinTHREE.build();

        Action ToSubmerse = drivetosubmerse.build();

        waitForStart();

        Actions.runBlocking(
               new SequentialAction(
                       ToBucket,
                       robot.Bin(),
                       new SleepAction(5),
                       robot.CO(),
                       new SleepAction(1),
                       robot.Rest(),


//                       robot.oclawopen(),
                       new SleepAction(.5),
//                       robot.RestArm(),
                        new SleepAction(10),
//                       ToRS,
//                       new SleepAction(3),
////                       robot.binRSpre(),
////                       new SleepAction(1),
////                       robot.binRSpost(),
////                       new SleepAction(2),
////                       robot.Hretract(),
////                       robot.Transfer(),
////                       new SleepAction(.1),
//                       Turn1,
//                       new SleepAction(3),
////                       robot.HBin(),
////                       robot.oclawopen(),
////                       new SleepAction(.5),
////                       robot.RestArm(),
////                       new SleepAction(3),
//                       ToMS,
//                       new SleepAction(3),
////                       robot.binMSpre(),
////                       new SleepAction(1),
////                       robot.binMSpost(),
////                       new SleepAction(2),
////                       robot.Hretract(),
////                       robot.Transfer(),
////                       new SleepAction(.1),
//                       Turn2,
//                       new SleepAction(3),
////                       robot.HBin(),
////                       robot.oclawopen(),
////                       new SleepAction(.5),
////                       robot.RestArm(),
////                       new SleepAction(3),
//                       ToLS,
//                       new SleepAction(3),
////                       robot.binLSpre(),
////                       new SleepAction(1),
////                       robot.binLSpost(),
////                       new SleepAction(2),
////                       robot.Transfer(),
////                       new SleepAction(.1),
//                       Turn3,
//                       new SleepAction(3),
////                       robot.HBin(),
////                       robot.oclawopen(),
////                       new SleepAction(.5),
////                       robot.RestArm(),
////                       new SleepAction(3)
                       ToSubmerse,
                       new SleepAction(3)
                ));
    }
}