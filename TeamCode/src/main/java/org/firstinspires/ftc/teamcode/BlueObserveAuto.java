package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

// RR-specific imports


@Config
@Autonomous(name="OAuto", group = "Observe")
public class BlueObserveAuto extends LinearOpMode {

    public static double ZOOM = 40;

    @Override
    public void runOpMode() {
        // Pose2d RedBucketPose = new Pose2d(12, -60, Math.toRadians(90));
        //Pose2d RedObservePose = new Pose2d(-18, -60, Math.toRadians(90));
        //Pose2d BlueBucketPose = new Pose2d(12, 60, Math.toRadians(-90));
        Pose2d BlueObservePose = new Pose2d(8, -60, Math.toRadians(270));



        hwRobot robot = new hwRobot();


        robot.init(hardwareMap);
        robot.RRest();

        robot.drive.pose = new Pose2d(8,-60,Math.toRadians(270));

        TrajectoryActionBuilder DriveToSubmersible = robot.drive.actionBuilder(new Pose2d(8, -60, Math.toRadians(270)))
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(0,-25),Math.toRadians(270), new TranslationalVelConstraint(ZOOM))
                .endTrajectory();

        TrajectoryActionBuilder Pushy = DriveToSubmersible.fresh()//.setTangent(Math.toRadians(180))
                .setTangent(Math.toRadians(-45))
                .splineToConstantHeading(new Vector2d(33,-30),Math.toRadians(90), new TranslationalVelConstraint(ZOOM))
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(40,-12), Math.toRadians(0), new TranslationalVelConstraint(ZOOM))
                .setTangent(0)
                .splineToConstantHeading(new Vector2d(48,-24), Math.toRadians(-90), new TranslationalVelConstraint(ZOOM))
                .lineToY(-44)
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(49,-12), Math.toRadians(0), new TranslationalVelConstraint(ZOOM))
                .splineToConstantHeading(new Vector2d(57,-24), Math.toRadians(-90),new TranslationalVelConstraint(ZOOM))
                .setTangent(Math.toRadians(-90))
                .lineToY(-44)
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(59,-12), Math.toRadians(0), new TranslationalVelConstraint(ZOOM))
                .splineToConstantHeading(new Vector2d(67,-24), Math.toRadians(-90), new TranslationalVelConstraint(ZOOM))
                .setTangent(Math.toRadians(-90))
                .lineToY(-44)
                .endTrajectory();

//        TrajectoryActionBuilder Deposit2 = Push3.fresh()//.setTangent(Math.toRadians(180))
//                .strafeToLinearHeading(new Vector2d(53,-42),Math.toRadians(-45))
//                .endTrajectory();
//
//        TrajectoryActionBuilder TurnObserve3 = Deposit2.fresh()
//                .turnTo(Math.toRadians(-40));
//
//        TrajectoryActionBuilder Drivewall = TurnObserve3.fresh()//.setTangent(Math.toRadians(180))
//                .strafeToSplineHeading(new Vector2d(62, -40),Math.toRadians(90))
//                .setTangent(Math.toRadians(90));
////                .lineToY(SHALLOW_END_POINT_RED.y);
//
//        TrajectoryActionBuilder Scorespecimen1 = Drivewall.fresh()//.setTangent(Math.toRadians(180))
//                .strafeToSplineHeading(new Vector2d(11,-40),Math.toRadians(270))
//                .setTangent(Math.toRadians(130))
//                .lineToY(-32.5);
//
//        TrajectoryActionBuilder Scorespecimen2 = Scorespecimen1.fresh()//.setTangent(Math.toRadians(180))
//                .strafeToSplineHeading(new Vector2d(62, -60),Math.toRadians(90))
//                .strafeToSplineHeading(new Vector2d(11,-40),Math.toRadians(270))
//                .setTangent(Math.toRadians(140))
//                .lineToY(-32.5);
//
//        TrajectoryActionBuilder Scorespecimen3 = Scorespecimen2.fresh()//.setTangent(Math.toRadians(180))
//                .strafeToSplineHeading(new Vector2d(62, -60),Math.toRadians(90))
//                .strafeToSplineHeading(new Vector2d(11,-40),Math.toRadians(270))
//                .setTangent(Math.toRadians(150))
//                .lineToY(-32.5);

        Action ToSubmerse = DriveToSubmersible.build();

        Action P = Pushy.build();

//        Action RS = ReadySpin.build();
//
//        Action P1 = Push1.build();
//
//        Action P2 = Push2.build();
//
//        Action P3 = Push3.build();

//        Action D2 = Deposit2.build();

        Action FullAuto = new SequentialAction(

                new ParallelAction(ToSubmerse, new SequentialAction(new SleepAction(.5), robot.Bar1())),
                new SleepAction(.3),
                robot.Bar2(),
                new SleepAction(.3),
                robot.CO(),
                new SleepAction(.1),
                robot.Rest(),
                P,

//                new ParallelAction(RS,  new SequentialAction(new SleepAction(.2), robot.CO())),
//                RS,
//                P1,
//                P2,
//                P3,

                new SleepAction(10),
                robot.Slide1(),
                new SleepAction(.1),
                robot.Adown(),
                new SleepAction(.3),
                robot.CC(),
                new SleepAction(.2),

//                new ParallelAction(P2, robot.AIn()),

                new ParallelAction(robot.CO(), robot.Slide2()),

//                P3,
                new SleepAction(.1),
                robot.Adown(),
                new SleepAction(.3),
                robot.CC(),
                new SleepAction(.2),

//                new ParallelAction(D2, robot.AIn()),

                new ParallelAction(robot.CO(), robot.Slide3())

        );

        waitForStart();

        Actions.runBlocking(
                new ParallelAction(
                        FullAuto,
                        robot.RotateAlways()
                )
        );
    }
}


