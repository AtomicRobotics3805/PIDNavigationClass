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
    private OpMode opmode;

    private double completedHeading;

    //Constructor
    public navigationMecanumPID(double[][] movementCommandArray, OpMode opmode, String configuredIMUname, DcMotor leftFront, DcMotor rightFront, DcMotor leftBack, DcMotor rightBack, int encoderTicksPerRev, double wheelDiameter) {
        this.opmode = opmode;
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

        opmode.telemetry.addData("Heading, Goal", "%1$s, %2$s", currentHeading(), movementArray[movementArrayStep][2]);
        opmode.telemetry.addData("Completed Heading", completedHeading);

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
        nextMovement();
    }

    public void moveNoStop(double TpForwards, double TpSlide) {
        yawAngle = AdafruitGyro.getYaw();
        loopPID(TpForwards, TpSlide, TpForwards >= TpSlide ? TpForwards : TpSlide);
    }

    public boolean rotateToAngle(double angle, double TpTurn) {
        yawAngle = AdafruitGyro.getYaw();
        if (yawAngle + 0.5 >= angle && yawAngle - 0.5 <= angle) {
            return true;
        } else {
            double error = yawAngle - setPoint;
            integral += error;
            double derivative = error - preError;
            double output = Range.clip((Kp * error) + (Ki * integral) + (Kd * derivative), -Math.abs(TpTurn), Math.abs(TpTurn));

            rightFront.setPower(-output);
            rightBack.setPower(-output);
            leftFront.setPower(output);
            leftBack.setPower(output);
            preError = error;
            return false;
        }
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
            loopPID(Tp, 0, Tp);
        } else {
            nextMovement();
        }
    }

    private void moveBackward(double goalDistanceInch, double Tp) { //Movement val == 2
        int distanceTraveled = (leftFront.getCurrentPosition() + rightBack.getCurrentPosition()) / 2;

        if (distanceTraveled > convertInchesToEncoderTicks(goalDistanceInch) + encoderPositionReference) {
            loopPID(Tp, 0, Tp);
        } else {
            nextMovement();
        }
    }

    private void rotateCW(double goalAngle, double Tp) { //Movement val == 3
        setPoint = goalAngle;
        if (yawAngle < setPoint) {
            loopPID(0, 0, Tp);
        } else {
            nextMovement();
        }
    }

    private void rotateCCW(double goalAngle, double Tp) { //Movement val == 4
        setPoint = goalAngle;
        if (yawAngle > setPoint) {
            loopPID(0, 0, Tp);
        } else {
            nextMovement();
        }
    }

    private void moveLeft(double goalDistanceInch, double Tp) { //Movement val == 2
        int distanceTraveled = (leftFront.getCurrentPosition() + rightBack.getCurrentPosition()) / 2;

        if (distanceTraveled < convertInchesToEncoderTicks(goalDistanceInch) + encoderPositionReference) {
            loopPID(0, Tp, Tp);
        } else {
            nextMovement();
        }
    }

    private void moveRight(double goalDistanceInch, double Tp) { //Movement val == 2
        int distanceTraveled = (leftFront.getCurrentPosition() + rightBack.getCurrentPosition()) / 2;

        if (distanceTraveled > convertInchesToEncoderTicks(goalDistanceInch) + encoderPositionReference) {
            loopPID(0, Tp, Tp);
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
        completedHeading = yawAngle;
        encoderPositionReference = (leftFront.getCurrentPosition() + rightBack.getCurrentPosition()) / 2;
        movementArrayStep++;
    }

    private void loopPID(double TpForwards, double TpSlide, double TpTurn) {
        double error = yawAngle - setPoint;
        integral += error;
        double derivative = error - preError;
        double output = Range.clip((Kp * error) + (Ki * integral) + (Kd * derivative), -Math.abs(TpTurn), Math.abs(TpTurn));

        rightFront.setPower(TpForwards - output - TpSlide);
        rightBack.setPower(TpForwards - output + TpSlide);
        leftFront.setPower(TpForwards + output + TpSlide);
        leftBack.setPower(TpForwards + output - TpSlide);
        preError = error;
    }
}
