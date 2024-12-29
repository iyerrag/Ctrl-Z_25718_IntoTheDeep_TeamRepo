package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Arrays;

public class actuators {
    private Servo wrist;
    private DcMotor elbow;
    private DcMotor leftSlide;
    private DcMotor rightSlide;
    private Servo beak;
    private Rev2mDistanceSensor lifterHeightSensor;
    private boolean closeState;
    private boolean holdState;




    public actuators(Servo wrist, Servo beakServo, DcMotor lifterLeft, DcMotor lifterRight, DcMotor elbow, DistanceSensor lifterHeightSensor){
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

        elbow.setDirection(DcMotor.Direction.FORWARD);
        elbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        wrist.setDirection(Servo.Direction.FORWARD);
        closeState = true;

        holdState = false;

        this.lifterHeightSensor = (Rev2mDistanceSensor) lifterHeightSensor;
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
            leftSlide.setPower(0.80);
            rightSlide.setPower(0.80);
        }
        else {
            rightSlide.setPower(0.80);
            leftSlide.setPower(0.80);
        }
    }

    public void resetLifters(){
        leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while(lifterHeightSensor.getDistance(DistanceUnit.MM) > 40){leftSlide.setPower(-1); rightSlide.setPower(-1);}
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

    public double getHeight(DistanceUnit unit) throws InterruptedException {
        double[] samples = new double[5];
        for(int i = 0; i < 5; i++){
            samples[i] = lifterHeightSensor.getDistance(unit);
            Thread.sleep(5);
        }

        /*for(int i = 0; i < 5; i++){
            for(int j = i + 1; j < 5; j++){
                if(samples[i] > samples[j]){
                    double temp = samples[i];
                    samples[i] = samples[j];
                    samples[j] = temp;
                }
            }
        }*/

        Arrays.sort(samples);
        return samples[2];
    }

    public void lift(double liftPwr) throws InterruptedException {
        if(!(holdState && liftPwr > 0)) {

            if (!(getHeight(DistanceUnit.CM) >= 65 && liftPwr > 0) && !(getHeight(DistanceUnit.CM) <= 4 && liftPwr < 0)) {

                leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                if ((int) (Math.random() * 2) == 0) {
                    leftSlide.setPower(liftPwr);
                    rightSlide.setPower(liftPwr);
                } else {
                    rightSlide.setPower(liftPwr);
                    leftSlide.setPower(liftPwr);
                }
                holdState = false;
            } else if (getHeight(DistanceUnit.CM) >= 65 && liftPwr > 0) {
                hold();
                holdState = true;
            } else {
                resetLifters();
                holdState = false;
            }
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
        liftTo(4800);
        Thread.sleep(2000);
        elbowTo(-650, 1);
        wristRotateTo(0);
    }

    public void moveToInsertPosition() throws InterruptedException {
        wristRotateTo(0.16);
        elbowTo(-2600, 1);
        resetLifters();
    }

    public void moveToPickupPosition() throws InterruptedException {
        if(closeState){changeClawState();}
        wristRotateTo(0.16);
        elbowTo(-2600, 1);
        wristRotateTo(0.49);
        resetLifters();
    }

    public void moveToPickupPositionAuto() throws InterruptedException {
        if(closeState){changeClawState();}
        wristRotateTo(0.16);
        elbowTo(-2675, 1);
        wristRotateTo(0.49);
        resetLifters();
    }

    public void moveToTransportPosition() throws InterruptedException {
        wristRotateTo(0.16);
        elbowTo(0, 1);
        resetLifters();
        wristRotateTo(0.49);

    }

    public void moveToHangInsertPosition() throws InterruptedException {
        liftTo(450);
        elbowTo(-1300, 1);
        wristRotateTo(0.49);
    }

    public void hang() throws InterruptedException {
        elbowTo(-1300, 1);
        wristRotateTo(0.49);
        liftTo(1300);
        elbowTo(-1300, 1);
    }

    public void moveToSpecimenExtractPos() throws InterruptedException{
        liftTo(350);
        elbowTo(0, 1);
        if(closeState){changeClawState();}
        wristRotateTo(.16);
    }

    public void moveToObservationDropOffPos() throws InterruptedException{
        resetLifters();
        elbowTo(0, 1);
        wristRotateTo(.16);
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

