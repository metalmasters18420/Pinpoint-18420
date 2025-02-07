package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.VariablesLift.Lbar;
import static org.firstinspires.ftc.teamcode.VariablesLift.Lbin;
import static org.firstinspires.ftc.teamcode.VariablesLift.Lhang1;
import static org.firstinspires.ftc.teamcode.VariablesLift.Lhang2;
import static org.firstinspires.ftc.teamcode.VariablesLift.Lin;
import static org.firstinspires.ftc.teamcode.VariablesLift.Lin2;
import static org.firstinspires.ftc.teamcode.VariablesLift.Lin3;
import static org.firstinspires.ftc.teamcode.VariablesLift.Lrest;
import static org.firstinspires.ftc.teamcode.VariablesLift.Lspeed;
import static org.firstinspires.ftc.teamcode.VariablesLift.Lwall;
import static org.firstinspires.ftc.teamcode.VariablesLift.THING;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Lift {

    private ElapsedTime timer = new ElapsedTime();
    public DcMotor left;
    public DcMotor right;
    int target = 0;
    double lastError = 0;
    double sum = 0;

    public Lift(DcMotor ll, DcMotor rl) {
        this.left = ll;
        this.right = rl;

        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setTargetPosition(Lrest);
        right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right.setPower(Lspeed);
        right.setDirection(DcMotorSimple.Direction.REVERSE);

        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left.setTargetPosition(Lrest);
        left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left.setPower(Lspeed);
    }

    public void LiftBin(){
        left.setTargetPosition(Lbin);
        right.setTargetPosition(Lbin);
    }
    public void LiftBar(){
        left.setTargetPosition(Lbar);
        right.setTargetPosition(Lbar);
    }
    public void LiftWall(){
        left.setTargetPosition(Lwall);
        right.setTargetPosition(Lwall);
    }
    public void LiftHang1(){
        left.setTargetPosition(Lhang1);
        right.setTargetPosition(Lhang1);
    }
    public void LiftHang2(){
        left.setTargetPosition(Lhang2);
        right.setTargetPosition(Lhang2);
    }
    public void LiftRest(){
        left.setTargetPosition(Lrest);
        right.setTargetPosition(Lrest);
    }
    public void LiftIn(){
        left.setTargetPosition(Lin);
        right.setTargetPosition(Lin);
    }
    public void LiftIn2(){
        left.setTargetPosition(Lin2);
        right.setTargetPosition(Lin2);
    }
    public void LiftIn3(){
        left.setTargetPosition(Lin3);
        right.setTargetPosition(Lin3);
    }
    public void Move(double y){
        left.setTargetPosition((int) (left.getCurrentPosition() + THING * -y));
        right.setTargetPosition((int) (right.getCurrentPosition() + THING * -y));
    }
}
