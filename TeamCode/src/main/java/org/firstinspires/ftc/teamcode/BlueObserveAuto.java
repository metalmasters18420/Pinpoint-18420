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

// RR-specific imports

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.opencv.core.Mat;


@Config
@Autonomous(name="Observe Auto", group = "Auto")
public class BlueObserveAuto extends LinearOpMode {
    @Override
    public void runOpMode() {
       // Pose2d RedBucketPose = new Pose2d(12, -60, Math.toRadians(90));
        //Pose2d RedObservePose = new Pose2d(-18, -60, Math.toRadians(90));
        //Pose2d BlueBucketPose = new Pose2d(12, 60, Math.toRadians(-90));
        Pose2d BlueObservePose = new Pose2d(-18, 60, Math.toRadians(-90));

        Vector2d DEEP_END_POINT_RED = new Vector2d(60,-60);
        Vector2d SHALLOW_END_POINT_RED = new Vector2d(48,-60);
        Vector2d DEEP_END_POINT_BLUE = new Vector2d(-60, 60);
        Vector2d SHALLOW_END_POINT_BLUE = new Vector2d(-48, 50);

        MecanumDrive drive = new MecanumDrive(hardwareMap, BlueObservePose);
        hwRobot robot = new hwRobot();
        //AutoArm arm = new AutoArm(hardwareMap);
//        AutoArm claw = new AutoArm(hardwareMap);
        //Claw claw = new Claw(hardwareMap);
        //Lift lift = new Lift(hardwareMap);

        robot.init(hardwareMap);
        TrajectoryActionBuilder DriveToSubmersible = drive.actionBuilder(BlueObservePose)//.setTangent(Math.toRadians(180))
                .strafeToSplineHeading(new Vector2d(11,-40),Math.toRadians(270))
                //.turn(Math.toRadians(180))
                .lineToY(-32.5);

        TrajectoryActionBuilder Observesample1 = DriveToSubmersible.fresh()//.setTangent(Math.toRadians(180))
                //.waitSeconds(2)
                .lineToY(-35)
                .setTangent(Math.toRadians(0))
                .strafeToSplineHeading(new Vector2d(28, -40),Math.toRadians(40));

        TrajectoryActionBuilder TurnObserve1 = Observesample1.fresh()
                .turnTo(Math.toRadians(-40));

        TrajectoryActionBuilder Observesample2 = TurnObserve1.fresh()//.setTangent(Math.toRadians(180))
                .strafeToSplineHeading(new Vector2d(38, -40),Math.toRadians(40));

        TrajectoryActionBuilder TurnObserve2 = Observesample2.fresh()
                .turnTo(Math.toRadians(-40));

        TrajectoryActionBuilder Observesample3 = TurnObserve2.fresh()//.setTangent(Math.toRadians(180))
                .strafeToSplineHeading(new Vector2d(48, -40),Math.toRadians(40));

        TrajectoryActionBuilder TurnObserve3 = Observesample3.fresh()
                .turnTo(Math.toRadians(-40));

        TrajectoryActionBuilder Drivewall = TurnObserve3.fresh()//.setTangent(Math.toRadians(180))
                .strafeToSplineHeading(new Vector2d(62, -40),Math.toRadians(90))
                .setTangent(Math.toRadians(90))
                .lineToY(SHALLOW_END_POINT_RED.y);

        TrajectoryActionBuilder Scorespecimen1 = Drivewall.fresh()//.setTangent(Math.toRadians(180))
                .strafeToSplineHeading(new Vector2d(11,-40),Math.toRadians(270))
                .setTangent(Math.toRadians(130))
                .lineToY(-32.5);

        TrajectoryActionBuilder Scorespecimen2 = Scorespecimen1.fresh()//.setTangent(Math.toRadians(180))
                .strafeToSplineHeading(new Vector2d(62, -60),Math.toRadians(90))
                .strafeToSplineHeading(new Vector2d(11,-40),Math.toRadians(270))
                .setTangent(Math.toRadians(140))
                .lineToY(-32.5);

        TrajectoryActionBuilder Scorespecimen3 = Scorespecimen2.fresh()//.setTangent(Math.toRadians(180))
                .strafeToSplineHeading(new Vector2d(62, -60),Math.toRadians(90))
                .strafeToSplineHeading(new Vector2d(11,-40),Math.toRadians(270))
                .setTangent(Math.toRadians(150))
                .lineToY(-32.5);

        Action ToSubmerse = DriveToSubmersible.build();

        Action LS = Observesample1.build();

        Action MS = Observesample2.build();

        Action RS = Observesample3.build();

        Action ToWall = Drivewall.build();

        Action Score1 = Scorespecimen1.build();

        Action Score2 = Scorespecimen2.build();

        Action Score3 = Scorespecimen3.build();

        Action LSturn = TurnObserve1.build();

        Action MSturn = TurnObserve2.build();

        Action RSturn = TurnObserve3.build();

        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                        ToSubmerse,
//                        robot.highbarpre(),
//                        robot.highbarpost(),
//                        robot.RestArm(),
                        LS,
//                        robot.barLSpre(),
//                        robot.barLSpost(),
                        LSturn,
                        MS,
//                        robot.barMSpre(),
//                        robot.barMSpost(),
                        MSturn,
                        RS,
//                        robot.barRSpre(),
//                        robot.barRSpost(),
                        RSturn,
//                        robot.wall(),
//                        robot.Hretract(),
                        ToWall,
                        Score1,
//                        robot.highbarpre(),
//                        robot.highbarpost(),
//                        robot.wall(),
                        Score2,
//                        robot.highbarpre(),
//                        robot.highbarpost(),
//                        robot.wall(),
                        Score3
//                        robot.highbarpre(),
//                        robot.highbarpost(),
//                        robot.RestArm()
                ));
    }
}


