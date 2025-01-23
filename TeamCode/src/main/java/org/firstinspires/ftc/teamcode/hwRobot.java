package org.firstinspires.ftc.teamcode;

//import static org.firstinspires.ftc.teamcode.ServoSpeed.ksu;
import static org.firstinspires.ftc.teamcode.VariablesArm.Arest;
import static org.firstinspires.ftc.teamcode.VariablesClaw.Cclose;
import static org.firstinspires.ftc.teamcode.VariablesClaw.Srest;
import static org.firstinspires.ftc.teamcode.VariablesClaw.Win;
import static org.firstinspires.ftc.teamcode.VariablesLift.Lrest;
import static org.firstinspires.ftc.teamcode.VariablesRotate.Rrest;


import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MecanumDrive;

public class hwRobot {
    HardwareMap hm = null;
    PinpointDrive drive = null;

    public DcMotor lLift = null;
    public DcMotor rLift = null;

    public Servo rrotate = null;
    public Servo lrotate = null;

    public Servo claw = null;
    public Servo wrist = null;
    public Servo spin = null;

    public Servo arm = null;

    public RevColorSensorV3 colorSensor = null;
    float gain = 2;
    public Servo Light = null;

    public Lift lift = null;
//    public ClawWheel clawWheel = null;
//    public ServoSpeed rotation = null;
    public hwRobot() {}

    public void init(HardwareMap hmap) {
        hm = hmap;

        lLift = hm.get(DcMotor.class, "LL"); //EH2 motor
        rLift = hm.get(DcMotor.class, "RL"); //EH3 motor
        rrotate = hm.get(Servo.class,"RR"); //CH0
        lrotate = hm.get(Servo.class,"LR"); //CH5
        Light = hm.get(Servo.class, "L"); //EH5
        claw = hm.get(Servo.class, "C"); //CH3
        wrist = hm.get(Servo.class, "W"); //CH2
        arm = hm.get(Servo.class, "A"); //CH4
        spin = hm.get(Servo.class, "S"); //CH1

        drive = new PinpointDrive(hmap,new Pose2d(0,0,0));

        lrotate.setPosition(Rrest);
        rrotate.setPosition(Rrest);
        lrotate.setDirection(Servo.Direction.REVERSE);

        Light.setPosition(.277);

        claw.setPosition(Cclose);
        claw.setDirection(Servo.Direction.REVERSE);

        wrist.setPosition(Win);
        wrist.setDirection(Servo.Direction.FORWARD);

        spin.setPosition(Srest);
        spin.setDirection(Servo.Direction.FORWARD);

        arm.setPosition(Arest);
        arm.setDirection(Servo.Direction.FORWARD);

        lift = new Lift(lLift,rLift);
            lLift.setTargetPosition(Lrest);
            rLift.setTargetPosition(Lrest);
    }
}