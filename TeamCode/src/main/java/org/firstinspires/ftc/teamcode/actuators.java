package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public class actuators {
    private Servo wrist;
    private DcMotor elbow;
    private DcMotor leftSlide;
    private DcMotor rightSlide;
    private Servo beak;
    private boolean closeState;



    public actuators(Servo wrist, Servo beakServo, DcMotor lifterLeft, DcMotor lifterRight, DcMotor elbow){
        this.wrist = wrist;
        this.elbow = elbow;
        this.beak = beakServo;
        leftSlide = lifterLeft;
        rightSlide = lifterRight;

        leftSlide.setDirection(DcMotor.Direction.REVERSE);
        leftSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightSlide.setDirection(DcMotor.Direction.FORWARD);
        rightSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        elbow.setDirection(DcMotor.Direction.REVERSE);
        elbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        wrist.setDirection(Servo.Direction.FORWARD);
        closeState = true;
    }

    public boolean state(){
        return closeState;
    }

    public void liftTo(int targetPos){

        leftSlide.setTargetPosition(targetPos);
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rightSlide.setTargetPosition(targetPos);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if((int) (Math.random() * 2) == 0){
            leftSlide.setPower(1);
            rightSlide.setPower(1);
        }
        else {
            rightSlide.setPower(1);
            leftSlide.setPower(1);
        }
    }

    public void resetLifters(){
        while(leftSlide.getCurrentPosition() != 0 || rightSlide.getCurrentPosition() !=0){/*wait*/}
        leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftSlide.setPower(0);
        rightSlide.setPower(0);
        leftSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void hold(){
        leftSlide.setTargetPosition(leftSlide.getCurrentPosition());
        rightSlide.setTargetPosition(rightSlide.getCurrentPosition());
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elbow.setTargetPosition(elbow.getCurrentPosition());
        elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if((int) (Math.random() * 2) == 0){
            leftSlide.setPower(1);
            rightSlide.setPower(1);
        }
        else {
            rightSlide.setPower(1);
            leftSlide.setPower(1);
        }

        elbow.setPower(1);
    }

    public void lift(double liftPwr){
        leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if ((int) (Math.random() * 2) == 0) {
            leftSlide.setPower(liftPwr);
            rightSlide.setPower(liftPwr);
        } else {
            rightSlide.setPower(liftPwr);
            leftSlide.setPower(liftPwr);
        }
    }

    public void rotateElbow(double rotatePwr){
        elbow.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        elbow.setPower(rotatePwr);
    }

    public void wristRotateTo(double targetAngle) throws InterruptedException {
        // Insert Conversion Formula
        double targetPos = targetAngle;
        wrist.setPosition(targetPos);
        Thread.sleep(500);
    }

    public void beakRotateTo(double targetAngle) throws InterruptedException {
        // Insert Conversion Formula
        double targetPos = targetAngle;
        beak.setPosition(targetPos);
    }

    public void changeClawState() throws InterruptedException {
        if(closeState){
            beakRotateTo(0.3);
            closeState = false;
        }

        else{
            beakRotateTo(0.65);
            closeState = true;
        }
        Thread.sleep(500);
    }

    public boolean getCloseState(){
        return closeState;
    }

    public void moveToHighBucketPosition() throws InterruptedException {
        liftTo(5000);
        Thread.sleep(2000);
        elbowTo(-450, 1);
        wristRotateTo(0);
    }

    public void moveToInsertPosition() throws InterruptedException {
        wristRotateTo(0.12);
        elbowTo(-2600, 1);
        liftTo(0);
        resetLifters();
    }

    public void moveToPickupPosition() throws InterruptedException {
        if(closeState){changeClawState();}
        wristRotateTo(0.12);
        elbowTo(-2600, 1);
        wristRotateTo(0.66);
        liftTo(0);
        resetLifters();
    }

    public void moveToTransportPosition() throws InterruptedException {
        beakRotateTo(0.6);
        wristRotateTo(0.12);
        elbowTo(0, 1);
        liftTo(0);
        resetLifters();
        wristRotateTo(0.66);

    }

    public void moveToHangInsertPosition() throws InterruptedException {
        liftTo(450);
        elbowTo(-1200, 1);
        wristRotateTo(0.66);
    }

    public void hang() throws InterruptedException {
        elbowTo(-1200, 1);
        wristRotateTo(0.66);
        liftTo(1300);
        elbowTo(-1000, 1);
        wristRotateTo(0.4);
    }

    public void moveToSpecimenExtractPos() throws InterruptedException{
        liftTo(0);
        resetLifters();
        elbowTo(0, 1);
        if(closeState){changeClawState();}
        wristRotateTo(.12);
    }

    // Add Extend & Rotate Functionality
    public void elbowTo(int targetPos, double maxPow){
        elbow.setTargetPosition(targetPos);
        elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elbow.setPower(maxPow);
    }

    public void elbowMove(String direction){
        elbow.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (direction.equals("forward")){
            elbow.setPower(.5);
        }
        else if (direction.equals("backward")){
            elbow.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            elbow.setPower(-.5);
        }
        else if (direction.equals("hold")){
            elbow.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            elbow.setPower(0);
        }
        else{};
    }

    public double getLiftPos(){
        return (leftSlide.getCurrentPosition() + rightSlide.getCurrentPosition()) / 2.0;
    }

    public double getElbowPos(){
        return (elbow.getCurrentPosition());
    }

}

