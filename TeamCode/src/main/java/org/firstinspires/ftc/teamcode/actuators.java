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

public class actuators{
    private Servo lwServo;
    private Servo rwServo;
    private Servo wrollServo;
    private DcMotor elbow;
    private DcMotor leftSlide;
    private DcMotor rightSlide;
    private Servo beak;
    private Rev2mDistanceSensor lifterHeightSensor;
    private RevTouchSensor lifterTouchSensor;
    private boolean closeState;
    private boolean holdState;
    private boolean highBasketState;
    private boolean hangInsertState;

    private static final double lwServo_OriginPos = 1.297 - 0.02;
    private static final double rwServo_OriginPos = -0.297 + 0.02;
    private static final double wrollServo_OriginPos = 0.86;


    public actuators(Servo lwServo, Servo rwServo, Servo wrollServo, Servo beakServo, DcMotor lifterLeft, DcMotor lifterRight, DcMotor elbow, DistanceSensor lifterHeightSensor, TouchSensor lifterTouchSensor){

        this.lwServo = lwServo;
        this.rwServo = rwServo;
        this.wrollServo = wrollServo;

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
        hangInsertState = false;

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

    public void wristRotateTo_Pitch(double targetAngle){
        double targetPos = (targetAngle / 268.4);
        if((int) (Math.random() * 2.0) == 0){
            lwServo.setPosition(lwServo_OriginPos - targetPos);
            rwServo.setPosition(rwServo_OriginPos + targetPos);
        }
        else{
            rwServo.setPosition(rwServo_OriginPos + targetPos);
            lwServo.setPosition(lwServo_OriginPos - targetPos);
        }
    }

    public void wristRotateTo_Roll(double targetAngle){
       double targetPos = wrollServo_OriginPos + (targetAngle / 270.0);
       wrollServo.setPosition(targetPos);
    }

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

    public double getWristAngle_Pitch(){
        double lwServo_PosEstimate = (lwServo_OriginPos - lwServo.getPosition()) * 268.4;
        double rwServo_PosEstimate = (rwServo.getPosition() - rwServo_OriginPos) * 268.4;
        return ((rwServo_PosEstimate + lwServo_PosEstimate) / 2.0);
    }

    public double getWristAngle_Roll(){
        return (wrollServo.getPosition() - wrollServo_OriginPos) * 270.0;
    }

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
        wristRotateTo_Pitch(90);
        while(!eqWT(getWristAngle_Pitch(), 90, 5)){}
        elbowRotateTo(90, 1);
        while(!eqWT(getElbowAngle(), 90, 5)){}
        liftTo(72);
        while(!eqWT(getLiftHeight(), 72, 1)){}
        elbowRotateTo(160, 1);
        wristRotateTo_Pitch(170);
        wristRotateTo_Roll(0);
        while(!eqWT(getElbowAngle(), 160, 1)){}
        while(!eqWT(getWristAngle_Pitch(), 170, 1)){}
        highBasketState = true;
        hangInsertState = false;
    }

    public void moveToInsertPosition(){
        liftTo(-12.5);
        wristRotateTo_Pitch(192.5);
        elbowRotateTo(-4, 1);
        wristRotateTo_Roll(0);
        resetLifters();
        highBasketState = false;
        hangInsertState = false;
    }

    public void moveToPickupPosition(){
        openBeak();
        moveToInsertPosition();
        elbowRotateTo(-7.5, 1);
        wristRotateTo_Pitch(97.5);
        highBasketState = false;
        hangInsertState = false;
    }

    public void submersiblePickup() throws InterruptedException {
        liftTo(0);
        elbowRotateTo(-20, 1);
        wristRotateTo_Pitch(110);
        resetLifters();
        Thread.sleep(200);
        closeBeak();
        Thread.sleep(200);
        elbowRotateTo(-5, 1);
        highBasketState = false;
        hangInsertState = false;
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
            wristRotateTo_Pitch(90);
            while(!eqWT(getElbowAngle(), 90, 5)){}
            while(!eqWT(getWristAngle_Pitch(), 90, 5)){}
            closeBeak();
            liftTo(0);
            resetLifters();
        }

        closeBeak();
        liftTo(0);
        elbowRotateTo(180, 1);
        wristRotateTo_Pitch(90);
        wristRotateTo_Roll(0);
        resetLifters();

        highBasketState = false;
        hangInsertState = false;
    }
    public void moveToStartingPosition(){
        liftTo(0);
        elbowRotateTo(198, 1);
        wristRotateTo_Pitch(10);
        closeBeak();
        resetLifters();
        highBasketState = false;
        hangInsertState = false;
    }
    public void moveToHangInsertPosition(){
        closeBeak();
        liftTo(0);
        elbowRotateTo(30, 1);
        wristRotateTo_Pitch(243.33);
        wristRotateTo_Roll(-180);
        resetLifters();
        highBasketState = false;
        hangInsertState = true;
    }

    public boolean getHangInsertState(){return hangInsertState;}

    public void hangRelease() throws InterruptedException {
        if(hangInsertState){
            moveToHangInsertPosition();
            openBeak();
            Thread.sleep(200);
            wristRotateTo_Pitch(135);
            elbowRotateTo(45, 1);
        }
        highBasketState = false;
        hangInsertState = false;
    }

    public void moveToSpecimenExtractPos(){
        liftTo(0);
        elbowRotateTo(190, 1);
        wristRotateTo_Pitch(160);
        wristRotateTo_Roll(0);
        while(!eqWT(getElbowAngle(), 190, 1)){}
        RobotLog.dd("ExtractPos", "Clearance 1: Elbow Condition Successful");
        while(!eqWT(getWristAngle_Pitch(), 160, 1)){RobotLog.dd("WristAngle", getWristAngle_Pitch() + "");}
        RobotLog.dd("ExtractPos", "Clearance 2: Wrist Condition Successful");
        openBeak();
        resetLifters();
        highBasketState = false;
        hangInsertState = false;
    }

    public void extract() throws InterruptedException {
        if(!getCloseState()){changeClawState();}
        liftTo(13);
        while ((getHeight(DistanceUnit.CM) <= 13)) {}
        highBasketState = false;
        hangInsertState = false;
    }

    public void initializePosition() throws InterruptedException {
        closeBeak();
        wristRotateTo_Pitch(150);
        Thread.sleep(200);
        wristRotateTo_Pitch(45);
        wristRotateTo_Roll(180);
        highBasketState = false;
        hangInsertState = false;
    }

    public void resetMotors() throws InterruptedException {
        liftTo(0);
        elbowRotateTo(193, 1);
        highBasketState = false;
        hangInsertState = false;
    }

    public void moveToObservationDropOffPos(){
        closeBeak();
        liftTo(0);
        elbowRotateTo(180, 1);
        wristRotateTo_Pitch(180);
        wristRotateTo_Roll(180);
        resetLifters();
        highBasketState = false;
        hangInsertState = false;
    }

}

