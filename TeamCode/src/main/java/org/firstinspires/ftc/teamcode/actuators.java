package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Arrays;

public class actuators extends armDifferential{
    private Servo differential_left;
    private Servo differential_right;
    private DcMotor elbow;
    private DcMotor leftSlide;
    private DcMotor rightSlide;
    private Servo beak;
    private Rev2mDistanceSensor lifterHeightSensor;
    private RevTouchSensor lifterTouchSensor;
    private boolean closeState;
    private boolean holdState;
    private boolean highBasketState;

    public actuators(Servo differential_left, Servo differential_right, Servo beakServo, DcMotor lifterLeft, DcMotor lifterRight, DcMotor elbow, DistanceSensor lifterHeightSensor, TouchSensor lifterTouchSensor){

        super(differential_left, differential_right, 0.4533,0.4533, 1);

        this.differential_left = differential_left;
        this.differential_right = differential_right;

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

        closeState = true;
        highBasketState = false;
        holdState = false;

        this.lifterHeightSensor = (Rev2mDistanceSensor) lifterHeightSensor;
        this.lifterTouchSensor = (RevTouchSensor) lifterTouchSensor;
    }

    public boolean state(){
        return closeState;
    }

    public void liftTo(double targetHeight){

        int targetPos = (int) (65.045045 * 0.72 * targetHeight);

        leftSlide.setTargetPosition(targetPos);
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rightSlide.setTargetPosition(targetPos);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        /*if(highBasketState){
            if((int) (Math.random() * 2) == 0){
                leftSlide.setPower(1);
                rightSlide.setPower(1);
            }
            else {
                rightSlide.setPower(1);
                leftSlide.setPower(1);
            }
        *///}
       // else {
            if ((int) (Math.random() * 2) == 0) {
                leftSlide.setPower(0.80);
                rightSlide.setPower(0.80);
            } else {
                rightSlide.setPower(0.80);
                leftSlide.setPower(0.80);
            }
        //}
    }

    public void resetLifters(){
        leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //if(lifterHeightSensor.getDistance(DistanceUnit.MM) > 60) {
        while(!lifterTouchSensor.isPressed()){
            leftSlide.setPower(-1);
            rightSlide.setPower(-1);
        }
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

    public void elbowRotateTo(double targetAngle, double maxPow){
        double targetPos = -2806 + 14.161 * targetAngle;
        elbow.setTargetPosition((int) targetPos);
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

    public void elbowRotate(double rotatePwr){
        elbow.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        elbow.setPower(rotatePwr);
    }

    /*public void wristRotateTo(double targetAngle){
        double targetPos = (targetAngle * .003770339) - 0.0382974576;
        wrist.setPosition(targetPos);
    }

    public void wristRotate(double rate){
        if(rate > 0){
            if(getWristPosition() + rate <= 1){
                wrist.setPosition(getWristPosition() + rate);
            }
        }
        else if(rate < 0){
            if(getWristPosition() + rate >= 0){
                wrist.setPosition(getWristPosition() + rate);
            }
        }
    }

    public void rotateTo(double targetAngle){
       double targetPos = 0.9 + (targetAngle * 0.00384);
       rotationServo.setPosition(targetPos);
    }

    public void rotate(double rate){
        if(rate > 0){
            if(getRotationServoPosition() + rate <= 1){
                rotationServo.setPosition(getRotationServoPosition() + rate);
            }
        }
        else if(rate < 0){
            if(getRotationServoPosition() + rate >= 0){
                rotationServo.setPosition(getRotationServoPosition() + rate);
            }
        }
    }*/

    public void beakRotateTo(double targetPos) {
        beak.setPosition(targetPos);
    }

    public void openBeak(){
        beakRotateTo(0.15);
        closeState = false;
    }

    public void closeBeak(){
        beakRotateTo(0.7);
        closeState = true;
    }

    public void changeClawState() throws InterruptedException {
        if(closeState){
            openBeak();
        }

        else{
            closeBeak();
        }

        Thread.sleep(200);
    }

    public boolean getCloseState(){
        return closeState;
    }

    public double getBeakPosition(){
        return beak.getPosition();
    }

    /*public double getWristPosition(){
        return wrist.getPosition();
    }

    public double getWristAngle(){
        return (getWristPosition() * 263.9728031) + 10.77211422;
    }

    public double getRotationServoPosition(){
        return rotationServo.getPosition();
    }*/

    public double getLiftPos(){
        return (leftSlide.getCurrentPosition() + rightSlide.getCurrentPosition()) / 2.0;
    }

    public double getLiftHeight(){
        return getLiftPos() * 0.015374/.72;
    }

    public double getElbowPos(){
        return (elbow.getCurrentPosition());
    }

    public double getElbowAngle(){
        return getElbowPos() * 0.0706 + 198.16;
    }

    public boolean eqWT(double val1, double val2, double e){
        return Math.abs(val1 - val2) <= e;
    }

    public void declareHighBucketStatusTrue(){highBasketState = true;}

    public void declareHighBucketStatusFalse(){highBasketState = false;}

    public void moveToHighBucketPosition(){
        closeBeak();
        setAngularPosition(90, getAngularPosition_Roll());
        while(!eqWT(getAngularPosition_Pitch(), 90, 5)){}
        elbowRotateTo(90, 1);
        while(!eqWT(getElbowAngle(), 90, 5)){}
        liftTo(72);
        while(!eqWT(getLiftHeight(), 72, 1)){}
        elbowRotateTo(160, 1);
        setAngularPosition(170, getAngularPosition_Roll());
        setAngularPosition(getAngularPosition_Pitch(), -180);
        while(!eqWT(getElbowAngle(), 160, 1)){}
        while(!eqWT(getAngularPosition_Pitch(), 170, 1)){}
        highBasketState = true;
    }

    public void moveToInsertPosition(){
        liftTo(0);
        setAngularPosition(180, getAngularPosition_Roll());
        elbowRotateTo(-4, 1);
        setAngularPosition(getAngularPosition_Pitch(), -180);
        resetLifters();
        highBasketState = false;
    }

    public void moveToPickupPosition(){
        openBeak();
        moveToInsertPosition();
        elbowRotateTo(2.5, 1);
        setAngularPosition(87.5, getAngularPosition_Roll());
        highBasketState = false;
    }

    public void submersiblePickup() throws InterruptedException {
        liftTo(0);
        elbowRotateTo(-5, 1);
        setAngularPosition(95, getAngularPosition_Roll());
        resetLifters();
        Thread.sleep(200);
        closeBeak();
        Thread.sleep(200);
        elbowRotateTo(5, 1);
        highBasketState = false;
    }

    /*public void moveToPickupPositionAuto() throws InterruptedException {
        if(closeState){changeClawState();}
        if(getRotationServoPosition() != 0.84){rotateTo(0.84);}
        wristRotateTo(0.16);
        elbowTo(-2775, 0.8);
        wristRotateTo(0.45);
        resetLifters();
    }*/

    public void moveToTransportPosition() throws InterruptedException {

        if(highBasketState){
            openBeak();
            Thread.sleep(200);
            elbowRotateTo(90, 1);
            setAngularPosition(90, getAngularPosition_Roll());
            while(!eqWT(getElbowAngle(), 90, 5)){}
            while(!eqWT(getAngularPosition_Pitch(), 90, 5)){}
            closeBeak();
            liftTo(0);
            resetLifters();
        }

        closeBeak();
        liftTo(0);
        elbowRotateTo(180, 1);
        setAngularPosition(90, -180);
        resetLifters();

        highBasketState = false;
    }
    public void moveToStartingPosition(){
        liftTo(0);
        elbowRotateTo(198, 1);
        setAngularPosition(10, 0);
        closeBeak();
        resetLifters();
        highBasketState = false;
    }
    public void moveToHangInsertPosition(){
        closeBeak();
        liftTo(0);
        elbowRotateTo(135, 1);
        setAngularPosition(55, -180);
        resetLifters();
        highBasketState = false;
    }

    public void hang(){
        moveToHangInsertPosition();
        liftTo(23);
        while(getLiftHeight() < 23){}
        openBeak();
        highBasketState = false;
    }

    public void moveToSpecimenExtractPos(){
        liftTo(0);
        elbowRotateTo(200, 1);
        setAngularPosition(160, -180);
        while(!eqWT(getElbowAngle(), 200, 1)){}
        while(!eqWT(getAngularPosition_Pitch(), 160, 1)){}
        openBeak();
        resetLifters();
        highBasketState = false;
    }

    public void extract() throws InterruptedException {
        if(!getCloseState()){changeClawState();}
        liftTo(13);
        while ((getHeight(DistanceUnit.CM) <= 13)) {}
    }

    public void initializePosition() throws InterruptedException {
        closeBeak();
        setAngularPosition(150, getAngularPosition_Roll());
        Thread.sleep(200);
        setAngularPosition(45, -180);
        highBasketState = false;
    }

    public void resetMotors() throws InterruptedException {
        liftTo(0);
        elbowRotateTo(193, 1);
        highBasketState = false;
    }

    public void moveToObservationDropOffPos(){
        closeBeak();
        liftTo(0);
        elbowRotateTo(180, 1);
        setAngularPosition(180, -180);
        resetLifters();
        highBasketState = false;
    }

}

