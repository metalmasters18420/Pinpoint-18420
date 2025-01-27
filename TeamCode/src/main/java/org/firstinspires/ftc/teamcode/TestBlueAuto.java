package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

// RR-specific imports

import org.firstinspires.ftc.teamcode.MecanumDrive;


@Config
@Disabled
@Autonomous(name="Test Blue Auto", group = "Auto")
public class TestBlueAuto extends LinearOpMode {
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
        //AutoArm arm = new AutoArm(hardwareMap);
//        AutoArm claw = new AutoArm(hardwareMap);
        //Claw claw = new Claw(hardwareMap);
        //Lift lift = new Lift(hardwareMap);




        TrajectoryActionBuilder drivetosubmersible = drive.actionBuilder(BlueObservePose)
                //.splineToLinearHeading(new Pose2d(-11,40,Math.toRadians(90)),Math.toRadians(-90))
                .setTangent(-1.23412150741)
                .lineToXSplineHeading(-11, Math.toRadians(-90))
                //.turn(Math.toRadians(180))
                .setTangent(Math.toRadians(90))
                .lineToY(28);
        TrajectoryActionBuilder observeSampleone = drivetosubmersible.fresh()
                .setTangent(Math.toRadians(90))
                .lineToY(35)
                .setTangent(Math.toRadians(-180))
                .splineTo(new Vector2d(-40,5),Math.toRadians(270))
                //.turnTo(Math.toRadians(180))
                .setTangent(Math.toRadians(0))
                .lineToX(-50)
                //.turnTo(Math.toRadians(270))
                .setTangent(Math.toRadians(90))
                .lineToY(SHALLOW_END_POINT_BLUE.y);
        TrajectoryActionBuilder observeSampletwo = observeSampleone.fresh()
                .setTangent(Math.toRadians(90))
                .lineToY(5)
                .setTangent(Math.toRadians(0))
                .lineToX(-58)
                .setTangent(Math.toRadians(90))
                .lineToY(SHALLOW_END_POINT_BLUE.y);
        TrajectoryActionBuilder prepareforwall = drivetosubmersible.fresh()
                .setTangent(Math.toRadians(90))
                .lineToY(30)
                .turn(Math.toRadians(90))
                .lineToX(-58);
        TrajectoryActionBuilder scorewallspecimenone = prepareforwall.fresh()
                .setTangent(Math.toRadians(-28))
                .lineToXSplineHeading(-11,Math.toRadians(-90))
                .setTangent(Math.toRadians(91))
                .lineToY(28);
        TrajectoryActionBuilder getspecimentwo = scorewallspecimenone.fresh()
                .setTangent(Math.toRadians(90))
                .lineToY(35)
                .setTangent(Math.toRadians(-28))
                .lineToXSplineHeading(-58,Math.toRadians(90));
        TrajectoryActionBuilder scorewallspecimentwo = getspecimentwo.fresh()
                .lineToXSplineHeading(-8,Math.toRadians(-90))
                .setTangent(Math.toRadians(91))
                .lineToY(28);
        TrajectoryActionBuilder park = drivetosubmersible.fresh()
                .setTangent(Math.toRadians(140))
                .lineToX(-47);

//                .lineToY(35)
//                .setTangent(Math.toRadians(-180))
//                .splineTo(new Vector2d(-35,5),Math.toRadians(270))
//                .turnTo(Math.toRadians(180))
//                .setTangent(Math.toRadians(0))
//                .lineToX(-45)
//                //.turnTo(Math.toRadians(270))
//                .setTangent(Math.toRadians(90))
//                .lineToY(SHALLOW_END_POINT_BLUE.y)
//                .setTangent(Math.toRadians(90))
//                .lineToY(5)
//                .setTangent(Math.toRadians(0))
//                .lineToX(-58)
//                .setTangent(Math.toRadians(90))
//                .lineToY(SHALLOW_END_POINT_BLUE.y);


//                .setTangent(Math.toRadians(90))
//                .lineToY(5)
//                .setTangent(Math.toRadians(0))
//                .lineToX(-62)
//                .setTangent(Math.toRadians(90))
//                .lineToY(SHALLOW_END_POINT_BLUE.y);



        Action drivetosubmersible1;
        drivetosubmersible1 = drivetosubmersible.build();

        Action observeSampleone1;
       observeSampleone1 = observeSampleone.build();

        Action observeSampletwo2;
        observeSampletwo2 = observeSampletwo.build();

        Action prepareforwall1 = prepareforwall.build();

        Action scorewallspecimenone1 = scorewallspecimenone.build();

        Action getspecimentwo1 = getspecimentwo.build();

        Action scorewallspecimentwo1 = scorewallspecimentwo.build();

        Action park1 = park.build();

        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                        //claw.rotateUp(),
                        drivetosubmersible1,
//                        claw.rotateUp(),
//                        claw.scoreSpecimen(),
                       // drivetospecimen1
                        observeSampleone1,
                        observeSampletwo2,
                       prepareforwall1,
                       // claw.prepareArmForWall(),
                        //wall code here\\
                        scorewallspecimenone1,
//                        claw.scoreSpecimen(),
                        getspecimentwo1,
                        scorewallspecimentwo1,
//                        claw.scoreSpecimen(),
                        park1  //help



                ));


//        waitForStart();
//        Actions.runBlocking(
//                new SequentialAction(
//                        claw.rotateUp(),
//                        claw.armHook(),
//                        claw.clawDrop(),
//                        claw.rotateDown(),
//                        claw.clawRestore()
//
//                )
//        );



    }
}


