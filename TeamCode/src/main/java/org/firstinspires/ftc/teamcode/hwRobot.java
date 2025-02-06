package org.firstinspires.ftc.teamcode;

//import static org.firstinspires.ftc.teamcode.ServoSpeed.ksu;
import static org.firstinspires.ftc.teamcode.VariablesArm.Abin;
import static org.firstinspires.ftc.teamcode.VariablesArm.Adown;
import static org.firstinspires.ftc.teamcode.VariablesArm.Ain;
import static org.firstinspires.ftc.teamcode.VariablesArm.Arest;
import static org.firstinspires.ftc.teamcode.VariablesArm.Awall;
import static org.firstinspires.ftc.teamcode.VariablesClaw.Cclose;
import static org.firstinspires.ftc.teamcode.VariablesClaw.Copen;
import static org.firstinspires.ftc.teamcode.VariablesClaw.Sin3;
import static org.firstinspires.ftc.teamcode.VariablesClaw.Srest;
import static org.firstinspires.ftc.teamcode.VariablesClaw.Wauto;
import static org.firstinspires.ftc.teamcode.VariablesClaw.Wbin;
import static org.firstinspires.ftc.teamcode.VariablesClaw.Win;
import static org.firstinspires.ftc.teamcode.VariablesClaw.Wrest;
import static org.firstinspires.ftc.teamcode.VariablesLift.Lin2;
import static org.firstinspires.ftc.teamcode.VariablesLift.Lrest;
import static org.firstinspires.ftc.teamcode.VariablesRotate.Rbin;
import static org.firstinspires.ftc.teamcode.VariablesRotate.Rin;
import static org.firstinspires.ftc.teamcode.VariablesRotate.Rrest;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.internal.stellaris.FlashLoaderResetCommand;
import org.firstinspires.ftc.teamcode.MecanumDrive;

public class hwRobot {
    HardwareMap hm = null;
    PinpointDrive drive = null;
    public AnalogInput armEncoder = null;
    public DcMotor lLift = null;
    public DcMotor rLift = null;
    public DcMotor Rotate = null;

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
        armEncoder = hm.get(AnalogInput.class,"ELC"); //CH Analog Input 0 or 1
        lLift = hm.get(DcMotor.class, "LL"); //EH2 motor
        rLift = hm.get(DcMotor.class, "RL"); //EH3 motor
        Rotate = hm.get(DcMotor.class, "R"); //EH3
        rrotate = hm.get(Servo.class,"RR"); //CH0
        lrotate = hm.get(Servo.class,"LR"); //CH5
        Light = hm.get(Servo.class, "L"); //EH5
        claw = hm.get(Servo.class, "C"); //CH3
        wrist = hm.get(Servo.class, "W"); //CH2
        arm = hm.get(Servo.class, "A"); //CH4
        spin = hm.get(Servo.class, "S"); //CH1

        drive = new PinpointDrive(hmap,new Pose2d(0,0,0));

        lrotate.setPosition(0.98);
        rrotate.setPosition(0.98);
        lrotate.setDirection(Servo.Direction.REVERSE);
        Rotate.setDirection(DcMotorSimple.Direction.FORWARD);

        Rotate.setPower(0);

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

    public class MoveClaw implements Action{
        double c;
        public MoveClaw(double c) {
            this.c = c;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            claw.setPosition(c);
            return false;
        }
    }

    public class MoveArm implements Action{
        double c;
        public MoveArm(double c) {
            this.c = c;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            arm.setPosition(c);
            return false;
        }
    }

    public class MoveWrist implements Action{
        double c;
        public MoveWrist(double c) {
            this.c = c;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            wrist.setPosition(c);
            return false;
        }
    }

    public class MoveRotate implements Action{
        double c;
        public MoveRotate(double c) {
            this.c = c;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            lrotate.setPosition(c);
            rrotate.setPosition(c);
            return false;
        }
    }

    public class MoveSpin implements Action{
        double c;
        public MoveSpin(double c) {
            this.c = c;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            spin.setPosition(c);
            return false;
        }
    }



    public Action CC(){
        return new MoveClaw(Cclose);
    }
    public Action CO(){
        return new MoveClaw(Copen);
    }
    public Action RBin(){
        return new MoveRotate(Rbin);
    }
    public Action RIn(){
        return new MoveRotate(Rin);
    }
    public Action RRest(){
        return new MoveRotate(Rrest);
    }
    public Action ABin(){
        return new MoveArm(Abin);
    }
    public Action AIn(){
        return new MoveArm(Ain);
    }
    public Action Adown(){
        return new MoveArm(Adown);
    }
    public Action Awall(){
        return new MoveArm(Awall);
    }
    public Action ARest(){
        return new MoveArm(Arest);
    }
    public Action WAuto(){return new MoveWrist(Wauto);}
    public Action WBin(){
        return new MoveWrist(Wbin);
    }
    public Action WIn(){
        return new MoveWrist(Win);
    }
    public Action Wrest(){
        return new MoveWrist(Wrest);
    }
    public Action Sin3(){
        return new MoveSpin(Sin3);
    }
    public Action Srest(){
        return new MoveSpin(Srest);
    }
    public Action LBin(){return new InstantAction(()->lift.LiftBin());}
    public Action LIn(){return new InstantAction(()->lift.LiftIn());}
    public Action LIn2(){return new InstantAction(()->lift.LiftIn2());}
    public Action LIn3(){return new InstantAction(()->lift.LiftIn3());}
    public Action LRest(){return new InstantAction(()->lift.LiftRest());}


    public Action Rest(){
        return new SequentialAction(
                //CO(),
                RRest(),
                ARest(),
                Wrest(),
                new SleepAction(0.5),
                LRest());
    }

    public Action RestFromIn(){
        return new SequentialAction(
                AIn(),
//                new SleepAction(1),
                LRest(),
                new SleepAction(0.5),
                Srest(),
                ARest(),
                Wrest(),
                RRest());
    }

    public Action Bin(){
        return new SequentialAction(
                RBin(),
                new SleepAction(.1),
                LBin(),
                new SleepAction(.65),
                ABin(),
                WBin()
        );
    }
    public Action In(){
        return new SequentialAction(
                RIn(),
                AIn(),
                WIn(),
                new SleepAction(.5),
                LIn()
        );
    }
    public Action In2(){
        return new SequentialAction(
                RIn(),
                AIn(),
                WIn(),
                new SleepAction(.5),
                LIn2()
        );
    }
    public Action In3(){
        return new SequentialAction(
                RIn(),
                AIn(),
                WIn(),
                Sin3(),
                new SleepAction(.5),
                LIn3()
        );
    }
}

