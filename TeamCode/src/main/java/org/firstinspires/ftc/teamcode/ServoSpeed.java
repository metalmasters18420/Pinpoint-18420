package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;

//@Config
//
//public class ServoSpeed {
//    private Servo rservo;
//    public Servo lservo;
//    private double Tpos;
//
//    public static double ksu = .2;
//    public static double ksd = .1;
//
//    public ServoSpeed(Servo s, Servo l, double p, double s1){
//        rservo = s;
//        lservo = l;
//        Tpos = p;
//        ksu = s1;
//
//        rservo.setDirection(Servo.Direction.REVERSE);
//        lservo.setDirection(Servo.Direction.FORWARD);
//
//        rservo.setPosition(Tpos);
//        lservo.setPosition(Tpos);
//    }
//
//    public void setTpos(double p){
//        Tpos = p;
//    }
//
//    public void update(){
//        double error = Tpos - rservo.getPosition();
//
//        if (rservo.getPosition() < Tpos) {
//            rservo.setPosition(rservo.getPosition() + error * ksu);
//            lservo.setPosition(lservo.getPosition() + error * ksu);
//        }
//
//        if (rservo.getPosition() > Tpos) {
//            rservo.setPosition(rservo.getPosition() - error * ksd);
//            lservo.setPosition(lservo.getPosition() - error * ksd);
//        }
//
//        if (Math.abs(error) < .005){
//            rservo.setPosition(Tpos);
//            lservo.setPosition(Tpos);
//        }
//    }
//}


