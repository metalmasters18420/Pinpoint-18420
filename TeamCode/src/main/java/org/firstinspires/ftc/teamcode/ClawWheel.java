package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class ClawWheel {
    Servo wrist = null;
    Servo claw = null;
    Servo arm = null;
    CRServo leftWheel = null;
    CRServo rightWheel = null;

    private final double STOP_POWER = 0;
    private final double INTAKE_POWER = 1;
    private final double OUTTAKE_POWER = -1;
    public static double CLOSED_CLAW_POS = 0;
    public static double OPENED_CLAW_POS = .08;

    public static double WRIST_INTAKE_POS = 1;
    public static double WRIST_INIT_POS = 0.5;
    public static double WRIST_DEPOSIT_POS = 0;

    public static double ARM_INTAKE_POS = 1;
    public static double ARM_INIT_POS = 0.5;
    public static double ARM_DEPOSIT_POS = 0;

    public ClawWheel(HardwareMap hwm){
        wrist = hwm.get(Servo.class, "wrist");
        claw = hwm.get(Servo.class, "claw");
        arm = hwm.get(Servo.class, "arm");
        leftWheel = hwm.get(CRServo.class, "leftintake");
        rightWheel = hwm.get(CRServo.class, "rightintake");

        wrist.setDirection(Servo.Direction.FORWARD);
        claw.setDirection(Servo.Direction.FORWARD);
        arm.setDirection((Servo.Direction.REVERSE));
        leftWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        rightWheel.setDirection(DcMotorSimple.Direction.FORWARD);

        leftWheel.setPower(STOP_POWER);
        rightWheel.setPower(STOP_POWER);

        rotateWristToInit();
        rotateArmToInit();
        openClaw();
    }

    void closeClaw(){
        claw.setPosition(CLOSED_CLAW_POS);
    }
    void openClaw(){
        claw.setPosition(OPENED_CLAW_POS);
    }

    void rotateArm(double pos){
        arm.setPosition(pos);
    }

    void rotateArmToIntake(){
        rotateArm(ARM_INTAKE_POS);
    }

    void rotateArmToDeposit(){
        rotateArm(ARM_DEPOSIT_POS);
    }

    void rotateArmToInit(){
        rotateArm(ARM_INIT_POS);
    }

    void rotateWrist(double pos){
        wrist.setPosition(pos);
    }

    void rotateWristToIntake(){
        rotateWrist(WRIST_INTAKE_POS);
    }

    void rotateWristToInit(){
        rotateWrist(WRIST_INIT_POS);
    }

    void rotateWristToDeposit(){
        rotateWrist(WRIST_DEPOSIT_POS);
    }

    void eatSpecimen(){
        leftWheel.setPower(INTAKE_POWER);
        rightWheel.setPower(INTAKE_POWER);
    }

    void spitSpecimen(){
        leftWheel.setPower(OUTTAKE_POWER);
        rightWheel.setPower(OUTTAKE_POWER);
    }

    void holdSpecimen(){
        leftWheel.setPower(STOP_POWER);
        rightWheel.setPower(STOP_POWER);
    }


}
