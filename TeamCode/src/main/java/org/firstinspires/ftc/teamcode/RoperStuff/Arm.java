package org.firstinspires.ftc.teamcode.RoperStuff;

import static org.firstinspires.ftc.teamcode.RoperStuff.RoperConstants.ARM_DEPOSIT_POSITION;
import static org.firstinspires.ftc.teamcode.RoperStuff.RoperConstants.ARM_GRAB_POSITION;
import static org.firstinspires.ftc.teamcode.RoperStuff.RoperConstants.ARM_MIDDLE_POSITION;
import static org.firstinspires.ftc.teamcode.RoperStuff.RoperConstants.ARM_MOVEMENT_TOLERANCE;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Arm {
    DcMotor arm = null;
    PIDFMotorController controller;
    double target = 0;

    public Arm(DcMotor a, PIDFMotorController controller){
        this.arm = a;
        this.controller = controller;
        a.setPower(0);
        a.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        a.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        a.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setTarget(double target){
        this.target = target;
    }

    public void updateArmPower(){
        arm.setPower(controller.calculate(target));
    }

    public double getPosition(){
        return arm.getCurrentPosition();
    }

    private class MoveArmToPosition implements Action{
        ElapsedTime timer;
        double target;

        public MoveArmToPosition(double target){
            this.target = target;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (timer == null){
                this.timer = new ElapsedTime();
            }
            setTarget(target);
            return Math.abs(arm.getCurrentPosition()-target) > ARM_MOVEMENT_TOLERANCE || timer.seconds()>2;
        }
    }

    public Action moveArmToGrab(){return new MoveArmToPosition(ARM_GRAB_POSITION);}
    public Action moveArmToDeposit(){return new MoveArmToPosition(ARM_DEPOSIT_POSITION);}
    public Action moveArmToMiddle(){return new MoveArmToPosition(ARM_MIDDLE_POSITION);}

}
