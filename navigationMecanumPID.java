package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;


public final class navigationMecanumPID {
    private AdafruitIMU AdafruitGyro; //Instance of AdafruitIMU
    private double yawAngle;  //Array to store IMU's output

    private DcMotor rightFront;
    private DcMotor rightBack;
    private DcMotor leftFront;
    private DcMotor leftBack;

    private double[][] movementArray;
    private int movementArrayStep = 0;

    //Variables for PID loop
    private double setPoint = 0;
    private double Kp;
    private double Ki;
    private double Kd;
    private double integral = 0;
    private double preError;


    private double encoderTicksPerInch;//= encoderTicksPerRev / (Math.PI * 3); //Variable to convert encoder ticks to inches
    private int encoderPositionReference; //Variable to use as reference for moveForward and moveBackward

    //Constructor
    public navigationMecanumPID(double[][] movementCommandArray, OpMode opmode, String configuredIMUname, DcMotor leftFront, DcMotor rightFront, DcMotor leftBack, DcMotor rightBack, int encoderTicksPerRev, double wheelDiameter) {
        AdafruitGyro = new AdafruitIMU(opmode, configuredIMUname);

        //Assign the DcMotor instances to those of the main class
        this.rightFront = rightFront;
        this.rightBack = rightBack;
        this.leftFront = leftFront;
        this.leftBack = leftBack;

        movementArray = movementCommandArray;

        encoderTicksPerInch = encoderTicksPerRev / (Math.PI * wheelDiameter);
    }

    //**********Public methods***********

    public void initialize() {
        AdafruitGyro.startIMU(); //Prepare the IMU for I2C communication

        encoderPositionReference = (leftFront.getCurrentPosition() + rightBack.getCurrentPosition()) / 2; //Set encoderPositionReference to the mean value of the current motor positions
    }

    public void tuneGains(double newKp, double newKi, double newKd) {
        //Set the gains for the PID loop
        Kp = newKp;
        Ki = newKi;
        Kd = newKd;
    }

    public void loopNavigation() {
        yawAngle = AdafruitGyro.getYaw();

        move((int) movementArray[movementArrayStep][0], movementArray[movementArrayStep][1], movementArray[movementArrayStep][2]);
    }

    public int navigationStep() {
        return movementArrayStep;
    }

    public int navigationType() {
        return (int) movementArray[movementArrayStep][0];
    }

    public double currentHeading() {
        return yawAngle;
    }

    public void forceNextMovement() {
        movementArrayStep++;
    }

    public void moveNoStop(double TpForwards, double TpSlide) {
        loopPID(TpForwards, TpSlide);
    }

    //**********Private methods**********

    private void move(int movementType, double Tp, double goalValue) {
        switch (movementType) {
            case 1:
                moveForward(goalValue, Tp);
                break;
            case 2:
                moveBackward(goalValue, Tp);
                break;
            case 3:
                rotateCW(goalValue, Tp);
                break;
            case 4:
                rotateCCW(goalValue, Tp);
                break;
            case 5:
                moveLeft(goalValue, Tp);
                break;
            case 6:
                moveRight(goalValue, Tp);
                break;
            case 10:
                fullStop();
                break;
            default:
                break;
        }
    }

    private void moveForward(double goalDistanceInch, double Tp) { //Movement val == 1
        int distanceTraveled = (leftFront.getCurrentPosition() + rightBack.getCurrentPosition()) / 2;

        if (distanceTraveled < convertInchesToEncoderTicks(goalDistanceInch) + encoderPositionReference) {
            loopPID(Tp, 0);
        } else {
            nextMovement();
        }
    }

    private void moveBackward(double goalDistanceInch, double Tp) { //Movement val == 2
        int distanceTraveled = (leftFront.getCurrentPosition() + rightBack.getCurrentPosition()) / 2;

        if (distanceTraveled > convertInchesToEncoderTicks(goalDistanceInch) + encoderPositionReference) {
            loopPID(Tp, 0);
        } else {
            nextMovement();
        }
    }

    private void rotateCW(double goalAngle, double Tp) { //Movement val == 3
        setPoint = goalAngle;
        if (yawAngle < setPoint) {
            loopPID(0, 0);
        } else {
            nextMovement();
        }
    }

    private void rotateCCW(double goalAngle, double Tp) { //Movement val == 4
        setPoint = goalAngle;
        if (yawAngle > setPoint) {
            loopPID(0, 0);
        } else {
            nextMovement();
        }
    }

    private void moveLeft(double goalDistanceInch, double Tp) { //Movement val == 2
        int distanceTraveled = (leftFront.getCurrentPosition() + rightBack.getCurrentPosition()) / 2;

        if (distanceTraveled < convertInchesToEncoderTicks(goalDistanceInch) + encoderPositionReference) {
            loopPID(0, Tp);
        } else {
            nextMovement();
        }
    }

    private void moveRight(double goalDistanceInch, double Tp) { //Movement val == 2
        int distanceTraveled = (leftFront.getCurrentPosition() + rightBack.getCurrentPosition()) / 2;

        if (distanceTraveled > convertInchesToEncoderTicks(goalDistanceInch) + encoderPositionReference) {
            loopPID(0, Tp);
        } else {
            nextMovement();
        }
    }

    private double convertInchesToEncoderTicks(double inches) {
        return encoderTicksPerInch * inches;
    }

    private void fullStop() {
        rightFront.setPower(0);
        rightBack.setPower(0);
        leftFront.setPower(0);
        leftBack.setPower(0);
    }

    private void nextMovement() {
        fullStop();
        encoderPositionReference = (leftFront.getCurrentPosition() + rightBack.getCurrentPosition()) / 2;
        movementArrayStep++;
    }

    /*private void loopPID(double Tp) {
        double error = yawAngle - setPoint;
        integral += error;
        double derivative = error - preError;
        double output = (Kp * error) + (Ki * integral) + (Kd * derivative);

        double leftOut = Range.clip(Tp + output, -0.5, 0.5);
        double rightOut = Range.clip(Tp - output, -0.5, 0.5);


        rightBack.setPower(rightOut);
        rightFront.setPower(rightOut);
        leftFront.setPower(leftOut);
        leftBack.setPower(leftOut);
        preError = error;
    }*/

    private void loopPID(double TpForwards, double TpSlide) {
        double error = yawAngle - setPoint;
        integral += error;
        double derivative = error - preError;
        double output = Range.clip((Kp * error) + (Ki * integral) + (Kd * derivative), -0.5, 0.5);

        rightFront.setPower(TpForwards - output - TpSlide);
        rightBack.setPower(TpForwards - output + TpSlide);
        leftFront.setPower(TpForwards + output + TpSlide);
        leftBack.setPower(TpForwards + output - TpSlide);
        preError = error;
    }
}