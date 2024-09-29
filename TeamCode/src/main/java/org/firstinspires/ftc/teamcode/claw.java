package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public class claw {
    private Servo claw;
    private Servo wrist;
    private DcMotor extender;
    private DcMotor leftSlide;
    private DcMotor rightSlide;



    public claw(Servo claw, Servo wrist, DcMotor lifterLeft, DcMotor lifterRight, DcMotor extension){
        this.claw = claw;
        this.wrist = wrist;
        extender = extension;
        leftSlide = lifterLeft;
        rightSlide = lifterRight;

        leftSlide.setDirection(DcMotor.Direction.FORWARD);
        leftSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightSlide.setDirection(DcMotor.Direction.FORWARD);
        rightSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void close(){
        claw.setPosition(0);
    }

    public void open(){
        claw.setPosition(1);
    }

    public void liftTo(int targetPos){

        leftSlide.setTargetPosition(targetPos);
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rightSlide.setTargetPosition(targetPos);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if((int) (Math.random() * 2) == 0){
            leftSlide.setPower(.5);
            rightSlide.setPower(.5);
        }
        else {
            rightSlide.setPower(.5);
            leftSlide.setPower(.5);
        }
    }

    // Add Extend & Rotate Functionality

    public int getLiftPos(){ return extender.getCurrentPosition();}
}
