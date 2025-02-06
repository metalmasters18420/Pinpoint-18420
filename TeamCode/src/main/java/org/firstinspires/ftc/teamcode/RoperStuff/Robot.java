package org.firstinspires.ftc.teamcode.RoperStuff;

import static org.firstinspires.ftc.teamcode.RoperStuff.RoperConstants.ARM_MIDPOINT_POS;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.PinpointDrive;

public class Robot {
    PinpointDrive drive = null;
    ArticulatingClaw grabber = null;
    Arm arm = null;
    ColorDetector colorDetector = null;

    Pose2d startPose = new Pose2d(0,0,0);

    public Robot (HardwareMap hardwareMap){
        drive = new PinpointDrive(hardwareMap, startPose);
        arm = new Arm(hardwareMap.get(DcMotor.class,"arm"),new PIDFMotorController(0,0,0,0,hardwareMap.get(DcMotor.class,"arm"),ARM_MIDPOINT_POS));
        grabber = new ArticulatingClaw(hardwareMap.get(Servo.class,"claw"),hardwareMap.get(Servo.class,"wrist"));
        colorDetector = new ColorDetector(hardwareMap);
    }

    public Action pickupSpecimen(){
        return new SequentialAction(
                grabber.moveWristToGrab(),
                grabber.grab(),
                grabber.moveWristToLevel()
        );
    }

    public Action closeClaw(){return grabber.grab();}
    public Action openClaw(){return grabber.open();}
    public Action rotateArmtoMiddle(){return arm.moveArmToMiddle();}
    public Action rotateArmtoDeposit(){return arm.moveArmToDeposit();}
    public Action rotateArmToGrab(){return arm.moveArmToGrab();}
}
