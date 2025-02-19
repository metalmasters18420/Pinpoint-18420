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

    public static double ZOOM = 65;
    public static double depth = -62;

    @Override
    public void runOpMode() {

        hwRobot robot = new hwRobot();

        robot.init(hardwareMap);
        robot.RRest();

        robot.drive.pose = new Pose2d(8,-64,Math.toRadians(270));

        TrajectoryActionBuilder DriveToSubmersible = robot.drive.actionBuilder(new Pose2d(8, -64, Math.toRadians(270)))
                .strafeToConstantHeading(new Vector2d(0,-24))
                .endTrajectory();

        TrajectoryActionBuilder Pushy = DriveToSubmersible.fresh()
                //move around submersible
                .strafeToConstantHeading(new Vector2d(33,-30), new TranslationalVelConstraint(ZOOM))
                //get into position
                .strafeToConstantHeading(new Vector2d(40,-14), new TranslationalVelConstraint(ZOOM))
                .strafeToConstantHeading(new Vector2d(46,-14), new TranslationalVelConstraint(ZOOM))
                //push first sample
                .strafeToConstantHeading(new Vector2d(44,-54), new TranslationalVelConstraint(ZOOM))
                //back up
                .strafeToConstantHeading(new Vector2d(48,-14), new TranslationalVelConstraint(ZOOM))
                .strafeToConstantHeading(new Vector2d(56,-14), new TranslationalVelConstraint(ZOOM))
                //push second sample
                .strafeToConstantHeading(new Vector2d(56,-56), new TranslationalVelConstraint(ZOOM))

                .endTrajectory();

        TrajectoryActionBuilder alignWall1 = Pushy.fresh()
                //go forward a bit to intake sample
                .strafeToConstantHeading(new Vector2d(56,depth))
                .endTrajectory();

        TrajectoryActionBuilder cycle1 = alignWall1.fresh()
                //go to submersible a little to the right of the preload
                .strafeToConstantHeading(new Vector2d(6,-24))
                .endTrajectory();

        TrajectoryActionBuilder shift1 = cycle1.fresh()
                //move the specs over!
                .strafeToConstantHeading(new Vector2d(2,-24))
                .endTrajectory();

        TrajectoryActionBuilder alignWall2 = shift1.fresh()
                //go back to wall
                .strafeToConstantHeading(new Vector2d(36,depth))
                .endTrajectory();

        TrajectoryActionBuilder cycle2 = alignWall2.fresh()
                //go to submersible
                .strafeToConstantHeading(new Vector2d(6,-24))
                .endTrajectory();

        TrajectoryActionBuilder shift2 = cycle2.fresh()
                //move the specs over!
                .strafeToConstantHeading(new Vector2d(2,-24))
                .endTrajectory();

        TrajectoryActionBuilder alignWall3 = shift2.fresh()
                //go back to wall
                .strafeToConstantHeading(new Vector2d(36,depth))
                .endTrajectory();

        TrajectoryActionBuilder cycle3 = alignWall3.fresh()
                //go to submersible
                .strafeToConstantHeading(new Vector2d(6,-24))
                .endTrajectory();

        TrajectoryActionBuilder shift3 = cycle3.fresh()
                //move the specs over!
                .strafeToConstantHeading(new Vector2d(2,-24))
                .endTrajectory();

        TrajectoryActionBuilder ending = shift3.fresh()
                //go to observation zone
                .strafeToConstantHeading(new Vector2d(36,depth))
                .endTrajectory();

        Action ToSubmerse = DriveToSubmersible.build();

        Action P = Pushy.build();

        Action align1 = alignWall1.build();

        Action score1 = cycle1.build();

        Action move1 = shift1.build();

        Action align2 = alignWall2.build();

        Action score2 = cycle2.build();

        Action move2 = shift2.build();

        Action align3 = alignWall3.build();

        Action score3 = cycle3.build();

        Action move3 = shift3.build();

        Action park = ending.build();

        Action FullAuto = new SequentialAction(

                //preload cycle
                new ParallelAction(ToSubmerse, new SequentialAction(new SleepAction(.5), robot.Bar1())),
                new SleepAction(.1),
                robot.Bar2(),
                new SleepAction(.3),
                robot.CO(),

                //go push!
                new ParallelAction(P, robot.Wall()),

                //pick up #1
                align1,
                new SleepAction(.25),
                robot.CC(),
                new SleepAction(.25),

                //make it clear the wall
                robot.ARest(),

                //score #1
                new ParallelAction(score1, new SequentialAction(new SleepAction(.5), robot.Bar1())),
                new SleepAction(.1),
                new ParallelAction(robot.Bar2(), move1),
                robot.CO(),

                //pick up #2
                robot.Wall(),
                align2,
                new SleepAction(.25),
                robot.CC(),
                new SleepAction(.25),

                //make it clear the wall
                robot.ARest(),

                //score #2
                new ParallelAction(score2, new SequentialAction(new SleepAction(.5), robot.Bar1())),
                new SleepAction(.1),
                new ParallelAction(robot.Bar2(), move2),
                robot.CO(),

                //pick up #3
                robot.Wall(),
                align3,
                new SleepAction(.25),
                robot.CC(),
                new SleepAction(.25),

                //make it clear the wall
                robot.ARest(),

                //score #3
                new ParallelAction(score3, new SequentialAction(new SleepAction(.5), robot.Bar1())),
                new SleepAction(.1),
                new ParallelAction(robot.Bar2(), move3),
                robot.CO(),

                //go park bozo
                new ParallelAction(park, new SequentialAction(new SleepAction(.2), robot.Rest()))


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


