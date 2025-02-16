package org.firstinspires.ftc.teamcode;

//import static org.firstinspires.ftc.teamcode.ServoSpeed.ksu;
import static org.firstinspires.ftc.teamcode.Rotation.degree_per_volt;
import static org.firstinspires.ftc.teamcode.Rotation.offset;
import static org.firstinspires.ftc.teamcode.Rotation.target;
import static org.firstinspires.ftc.teamcode.VariablesArm.Aauto;
import static org.firstinspires.ftc.teamcode.VariablesArm.Abar;
import static org.firstinspires.ftc.teamcode.VariablesArm.Abar2;
import static org.firstinspires.ftc.teamcode.VariablesArm.Abin;
import static org.firstinspires.ftc.teamcode.VariablesArm.Adown;
import static org.firstinspires.ftc.teamcode.VariablesArm.Ain;
import static org.firstinspires.ftc.teamcode.VariablesArm.Arest;
import static org.firstinspires.ftc.teamcode.VariablesArm.Awall;
import static org.firstinspires.ftc.teamcode.VariablesClaw.Cclose;
import static org.firstinspires.ftc.teamcode.VariablesClaw.Copen;
import static org.firstinspires.ftc.teamcode.VariablesClaw.Sin3;
import static org.firstinspires.ftc.teamcode.VariablesClaw.Srest;
import static org.firstinspires.ftc.teamcode.VariablesClaw.Sslide1;
import static org.firstinspires.ftc.teamcode.VariablesClaw.Sslide2;
import static org.firstinspires.ftc.teamcode.VariablesClaw.Sslide3;
import static org.firstinspires.ftc.teamcode.VariablesClaw.Wauto;
import static org.firstinspires.ftc.teamcode.VariablesClaw.Wbar;
import static org.firstinspires.ftc.teamcode.VariablesClaw.Wbar2;
import static org.firstinspires.ftc.teamcode.VariablesClaw.Wbin;
import static org.firstinspires.ftc.teamcode.VariablesClaw.Win;
import static org.firstinspires.ftc.teamcode.VariablesClaw.Wrest;
import static org.firstinspires.ftc.teamcode.VariablesLift.Lrest;
import static org.firstinspires.ftc.teamcode.VariablesRotate.Rbar;
import static org.firstinspires.ftc.teamcode.VariablesRotate.Rbar2;
import static org.firstinspires.ftc.teamcode.VariablesRotate.Rin;
import static org.firstinspires.ftc.teamcode.VariablesRotate.Rrest;
//import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.Rotate


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


// intake voltage: .098
// resting voltage: 1


public class hwRobot {
    HardwareMap hm = null;
    PinpointDrive drive = null;
    public AnalogInput armEncoder = null;
    public Rotation rotation = null;

    public DcMotor lLift = null;
    public DcMotor rLift = null;
    public DcMotorEx Rotate = null;

    public Servo rrotate = null;
    public Servo lrotate = null;

    public Servo claw = null;
    public Servo wrist = null;
    public Servo spin = null;

    public PIDController controller = null;

    public Servo arm = null;

    public RevColorSensorV3 colorSensor = null;
    float gain = 2;
    public Servo Light = null;

    public Lift lift = null;
    public hwRobot() {}



    public void init(HardwareMap hmap) {
        hm = hmap;
        armEncoder = hm.get(AnalogInput.class,"ELC"); //CH Analog Input 0 or 1
        lLift = hm.get(DcMotor.class, "LL"); //EH0 motor
        rLift = hm.get(DcMotor.class, "RL"); //EH1 motor
        Rotate = hm.get(DcMotorEx.class, "R"); //EH2 motor
        Light = hm.get(Servo.class, "L"); //EH5
        claw = hm.get(Servo.class, "C"); //CH3
        wrist = hm.get(Servo.class, "W"); //CH2
        arm = hm.get(Servo.class, "A"); //CH4
        spin = hm.get(Servo.class, "S"); //CH1

        drive = new PinpointDrive(hmap,new Pose2d(0,0,0));

        Light.setPosition(.277);

        claw.setDirection(Servo.Direction.REVERSE);
        claw.setPosition(Cclose);

        wrist.setDirection(Servo.Direction.FORWARD);
        wrist.setPosition(Win);

        spin.setDirection(Servo.Direction.FORWARD);
        spin.setPosition(Srest);

        arm.setDirection(Servo.Direction.FORWARD);
        arm.setPosition(Arest);

        lift = new Lift(lLift,rLift);
            lLift.setTargetPosition(Lrest);
            rLift.setTargetPosition(Lrest);

        rotation = new Rotation(Rotate, armEncoder);
           rotation.RotateRest();
           Rotate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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

//    public class MoveRotate implements Action{
//        double c;
//        public MoveRotate(double c) {
//            this.c = c;
//        }
//
//        @Override
//        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//            lrotate.setPosition(c);
//            rrotate.setPosition(c);
//            return false;
//        }
//    }

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

    public class Rotate implements Action{
        double t;
        public Rotate(double t) {this.t = t;}

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            target = t + offset;
            rotation.loop();
            return Math.abs(target - rotation.ELC.getVoltage() * degree_per_volt) > 10;
        }
    }

    public class Rotate1 implements Action{
        public Rotate1(){}

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            rotation.loop();
            return true;
        }
    }
    public Action RotateAlways(){return new Rotate1();}
    public Action RRest() {return new Rotate(Rrest);}
    public Action RIn(){return new Rotate(Rin);}
    public Action CC(){
        return new MoveClaw(Cclose);
    }
    public Action CO(){
        return new MoveClaw(Copen);
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
    public Action Aauto(){return new MoveArm(Aauto);}
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
    public Action ABar1(){return new MoveArm(Abar);}
    public Action WBar1(){return new MoveWrist(Wbar);}
    public Action Rbar1(){return new Rotate(Rbar);}
    public Action LBar1(){return new InstantAction(() -> lift.LiftBar());}
    public Action ABar2(){return new MoveArm(Abar2);}
    public Action WBar2(){return new MoveWrist(Wbar2);}
    public Action RBar2(){return new Rotate(Rbar2);}
    public Action LBar2(){return new InstantAction(() -> lift.LiftBar2());}

    public Action LSlide1(){return new InstantAction(() -> lift.LiftSlide1());}
    public Action LSlide2(){return new InstantAction(() -> lift.LiftSlide2());}
    public Action LSlide3(){return new InstantAction(() -> lift.LiftSlide3());}

    public Action SSlide1(){return new MoveSpin(Sslide1);}
    public Action SSlide2(){return new MoveSpin(Sslide2);}
    public Action SSlide3(){return new MoveSpin(Sslide3);}




    public Action Rest(){
        return new SequentialAction(
                CO(),
                RRest(),
                ARest(),
                Wrest(),
                new SleepAction(0.5),
                LRest());
    }

    public Action RestFromIn(){
        return new SequentialAction(
                AIn(),
//                new SleepAction(.2),
                LRest(),
                new SleepAction(0.3),
                Srest(),
                ARest(),
                Wrest(),
                RRest());
    }

    public Action Bin(){
        return new SequentialAction(
//                RRest(),
//                new SleepAction(.1),
                LBin(),
                new SleepAction(.65),
                ABin(),
                WBin()
        );
    }
    public Action In(){
        return new SequentialAction(
                AIn(),
                WIn(),
//                new SleepAction(.1),
                RIn(),
                LIn()
        );
    }
    public Action In2(){
        return new SequentialAction(
                AIn(),
                WIn(),
                RIn(),
//                new SleepAction(.5),
                LIn2()
        );
    }
    public Action In3(){
        return new SequentialAction(
                AIn(),
                WIn(),
                RIn(),
                Sin3(),
//                new SleepAction(.5),
                LIn3()
        );
    }

    public Action Bar1(){
        return new SequentialAction(
                ABar1(),
                WBar1(),
                Rbar1(),
                LBar1()
        );
    }

    public Action Bar2(){
        return new SequentialAction(
                ABar2(),
                WBar2(),
                RBar2(),
                LBar2()
        );
    }

    public Action Slide1(){
        return new SequentialAction(
                AIn(),
                WIn(),
                RIn(),
                SSlide1(),
                LSlide1()
        );
    }

    public Action Slide2(){
        return new SequentialAction(
                AIn(),
                WIn(),
                RIn(),
                SSlide2(),
                LSlide2()
        );
    }

    public Action Slide3(){
        return new SequentialAction(
                AIn(),
                WIn(),
                RIn(),
                SSlide3(),
                LSlide3()
        );
    }
}