package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;

@Autonomous(name = "ColorStoneLeft", group = "Iterative OpMode")

public class AutoTest2 extends OpMode {
    DcMotor frontRight, frontLeft, backRight, backLeft, extendArm;
    CRServo claw1, claw2;
    ColorSensor mrSensor;
    Servo mrServo;

    oneServo sensorDown;
    timeState forward;
    driveState strafeLeft;
    extendArmState reachOut;
    private StateMachine machine;
    clampDriveState strafeRight;
    timeState turnLeft;
    CRServoState2 close;
    CRServoState open;
    CRServoState2 close2;
    timeState park;
    ColorState colorState;
    ColorState colorState2;
    timeState backwards;
    timeState backwards2;
    driveState strafeLeftAgain;
    timeState stop;
    extendArmState raiseArm;
    DcMotor raiser;
    extendArmState reachOut2;
    extendArmState retract;
    CRServoState2 close3;
    clampDriveState strafeLeft3;
    CRServoState2 close4;
    clampDriveState strafeRight2;
    oneServo up;

    ArrayList<Servo> servoPickUp= new ArrayList<Servo>();

    ArrayList<DcMotor> motors = new ArrayList<DcMotor>();
    ArrayList<CRServo> crServos;

    {
        crServos = new ArrayList<CRServo>();
    }


    @Override
    public void init() {

        frontRight=hardwareMap.dcMotor.get("front right");
        frontLeft=hardwareMap.dcMotor.get("front left");
        backRight=hardwareMap.dcMotor.get("back right");
        backLeft=hardwareMap.dcMotor.get("back left");
        mrSensor=hardwareMap.get(ColorSensor.class, "mrSensor");
        mrServo=hardwareMap.servo.get("mrServo");
        raiser=hardwareMap.dcMotor.get("raise arm 2");
        extendArm=hardwareMap.dcMotor.get("extend arm");

        claw1= hardwareMap.crservo.get("claw 1");
        claw2 = hardwareMap.crservo.get("claw 2");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        motors.add(frontLeft);
        motors.add(frontRight);
        motors.add(backLeft);
        motors.add(backRight);

        crServos.add(claw1);
        crServos.add(claw2);

        sensorDown = new oneServo(500, 0, mrServo);
        strafeLeft = new driveState(30, .9, motors, "strafeLeft"); //before 28
        colorState = new ColorState(motors, mrSensor,"forward","alpha", 4500);
        backwards = new timeState(600, .5, motors, "backward");
        colorState2 = new ColorState(motors, mrSensor,"forward", "alpha",4500);
        backwards2 = new timeState(300, .5, motors, "backward");
        stop = new timeState(500, 0, motors, "forward");
        strafeLeftAgain = new driveState(14, .9, motors, "strafeLeft");
        reachOut = new extendArmState(1200, -.5, extendArm);
        close = new CRServoState2(1500,-1,1, crServos);
        retract = new extendArmState(600, .5, extendArm);
        close3 = new CRServoState2(1500,-1,1, crServos);
        strafeRight = new clampDriveState(16,.9, motors, "strafeRight", -1, 1, crServos);
        close2 = new CRServoState2(1500, -1, 1, crServos);
        turnLeft = new timeState(2100, .5, motors, "turnLeft"); //TODO: maybe lower time?
        strafeLeft3 = new clampDriveState(33, .5, motors, "strafeLeft", -1,1,crServos);
        strafeRight2 = new clampDriveState(25, .5, motors, "strafeRight", -1,1,crServos);
        close4 = new CRServoState2(1500,-1,1, crServos);


        

        forward = new timeState(2500, .5, motors, "forward");
        open = new CRServoState(700, 1, -1, crServos);
        park = new timeState (1500, .5, motors, "backward");
        up = new oneServo(500, .8, mrServo);


        sensorDown.setNextState(strafeLeft);
        strafeLeft.setNextState(colorState);
        colorState.setNextState(backwards);
        backwards.setNextState(colorState2);
        colorState2.setNextState(backwards2);
        backwards2.setNextState(stop);
        stop.setNextState(strafeLeftAgain);
        strafeLeftAgain.setNextState(up);
        up.setNextState(reachOut);
        reachOut.setNextState(close);
        close.setNextState(strafeRight);
        strafeRight.setNextState(close2);
        close2.setNextState(turnLeft);
        turnLeft.setNextState(close4);
        close4.setNextState(strafeLeft3);
        strafeLeft3.setNextState(strafeRight2);
        strafeRight2.setNextState(forward);
        forward.setNextState(open);
        open.setNextState(park);
        park.setNextState(null);


    }
    @Override
    public void start(){


        machine = new StateMachine(sensorDown);
    }
    @Override
    public void loop()  {


        machine.update();

    }

    @Override
    public void stop() {
    }


}

