package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

// RR-specific imports

//import org.firstinspires.ftc.teamcode.AutoHardware.AutoArm;


@Config
@Autonomous(name="BPushbot", group = "Bucket")
public class BucketFivePushbot extends LinearOpMode {
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
        robot.RRest();


        robot.drive.pose = new Pose2d(-40,-60,Math.toRadians(0));



        TrajectoryActionBuilder drivetobucket = robot.drive.actionBuilder(new Pose2d(-40, -60, Math.toRadians(0)))
                .strafeToLinearHeading(new Vector2d(-52, -49),Math.toRadians(45))
                .endTrajectory();
        TrajectoryActionBuilder rightsample = drivetobucket.fresh()
                .turnTo(Math.toRadians(77.5))
                .endTrajectory();
        TrajectoryActionBuilder turntobinONE = rightsample.fresh()
                .turnTo(Math.toRadians(45))
                .endTrajectory();
        TrajectoryActionBuilder middlesample = turntobinONE.fresh()
                .turnTo(Math.toRadians(98.5))
                .endTrajectory();
        TrajectoryActionBuilder turntobinTWO = middlesample.fresh()
                .turnTo(Math.toRadians(45))
                .endTrajectory();
        TrajectoryActionBuilder leftsample = turntobinTWO.fresh()
                .turnTo(Math.toRadians(117.5))
                .endTrajectory();
        TrajectoryActionBuilder turntobinTHREE = leftsample.fresh()
                .turnTo(Math.toRadians(45))
                .endTrajectory();
        TrajectoryActionBuilder fifth = turntobinTHREE.fresh()
                .strafeToLinearHeading(new Vector2d(-34, -60), Math.toRadians(0))
                .endTrajectory();
        TrajectoryActionBuilder drivetobucket2 = fifth.fresh()
                .strafeToSplineHeading(new Vector2d(-52, -49),Math.toRadians(45))
                .endTrajectory();
        TrajectoryActionBuilder drivetosubmerse = drivetobucket2.fresh()
                .strafeToLinearHeading(new Vector2d(-14, 0), Math.toRadians(180))
                .endTrajectory();

        Action ToBucket = drivetobucket.build();

        Action ToRS = rightsample.build();

        Action Turn1 = turntobinONE.build();

        Action ToMS = middlesample.build();

        Action Turn2 = turntobinTWO.build();

        Action ToLS = leftsample.build();

        Action Turn3 = turntobinTHREE.build();

        Action s5 = fifth.build();

        Action b2 = drivetobucket2.build();

        Action ToSubmerse = drivetosubmerse.build();

        Action FullAuto = new SequentialAction(

                ToBucket,
                robot.Bin(),
                new SleepAction(.3),
                robot.CO(),
                new SleepAction(0.2),
                robot.Rest(),


                ToRS,
                robot.In(),
                new SleepAction(0.4),
                robot.Adown(),
                new SleepAction(0.2),
                robot.CC(),
                new SleepAction(0.2),
                robot.RestFromIn(),



                new ParallelAction(robot.Bin(), Turn1),
                new SleepAction(.3),
                robot.CO(),
                new SleepAction(0.2),
                robot.Rest(),


                ToMS,
                robot.In2(),
                new SleepAction(0.4),
                robot.Adown(),
                new SleepAction(0.2),
                robot.CC(),
                new SleepAction(0.2),
                robot.RestFromIn(),


                new ParallelAction(robot.Bin(), Turn2),
                new SleepAction(.3),
                robot.CO(),
                new SleepAction(0.2),
                robot.Rest(),


                ToLS,
                robot.In3(),
                new SleepAction(0.4),
                robot.Adown(),
                new SleepAction(0.2),
                robot.CC(),
                new SleepAction(0.2),
                robot.RestFromIn(),


                new ParallelAction(robot.Bin(), Turn3),
                new SleepAction(.3),
                robot.CO(),
                new SleepAction(0.2),
                robot.Rest(),



                s5,
                robot.In(),
                new SleepAction(.4),
                robot.Adown(),
                new SleepAction(.2),
                robot.CC(),
                new SleepAction(.2),
                robot.RestFromIn(),

                b2,
                robot.Bin(),
                new SleepAction(.3),
                robot.CO(),
                new SleepAction(0.2),
                robot.Rest(),

                new ParallelAction(ToSubmerse, robot.Aauto(), robot.RRest(), robot.WAuto()),
                new SleepAction(4)
        );


                waitForStart();

        Actions.runBlocking(new ParallelAction(
                FullAuto,
                robot.RotateAlways()));
    }
}