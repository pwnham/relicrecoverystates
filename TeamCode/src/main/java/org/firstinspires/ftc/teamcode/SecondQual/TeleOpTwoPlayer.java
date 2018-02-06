package org.firstinspires.ftc.teamcode.SecondQual;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by moham on 1/26/18.
 */
@TeleOp
public class TeleOpTwoPlayer extends Robot {

    private double drivePower, driveTurn;

    @Override
    public void init(){
        super.init();
    }

    @Override
    public void loop(){
        drivePower = gamepad1.left_stick_y;
        driveTurn = .5 * gamepad1.right_stick_x;

        if (gamepad1.left_bumper) {
            rightBack.setPower(-(drivePower + driveTurn) * .3);
            rightFront.setPower(-(drivePower + driveTurn) * .3);
            leftBack.setPower(-(drivePower - driveTurn) * .3);
            leftFront.setPower(-(drivePower - driveTurn) * .3);
        } else {
            rightBack.setPower(-(drivePower + driveTurn));
            rightFront.setPower(-(drivePower + driveTurn));
            leftBack.setPower(-(drivePower - driveTurn));
            leftFront.setPower(-(drivePower - driveTurn));
        }

        //intake in
        if (gamepad2.a) {
            intakeLeft.setPower(1);
            intakeRight.setPower(1);
        } else if (gamepad2.b) {//intake out
            intakeLeft.setPower(-1);
            intakeRight.setPower(-1);
        } else {
            intakeLeft.setPower(0);
            intakeRight.setPower(0);
        }

        //lower slide
        if (gamepad2.x) {
            slideLeft.setPower(.65);
            slideRight.setPower(.65);
        } else if (gamepad2.y) { // raise slide
            slideLeft.setPower(-.99);
            slideRight.setPower(-.99);
        } else {
            slideLeft.setPower(0);
            slideRight.setPower(0);
        }

        //flip tray
        if (gamepad2.dpad_up) {
            flipLeft.setPosition(flipLeftUp);
            flipRight.setPosition(flipRightUp);
        } else if (gamepad2.dpad_down) {//lower tray
            flipLeft.setPosition(flipLeftDown);
            flipRight.setPosition(flipRightDown);
        } else if (gamepad2.dpad_left) {
            flipLeft.setPosition(flipLeftMiddle);
            flipRight.setPosition(flipRightMiddle);
        }

        //jewel arm
        if (gamepad1.dpad_up) {
            jewelArm.setPosition(jewelArmUp);
        } else if (gamepad1.dpad_down) {
            jewelArm.setPosition(jewelArmDown);
        }

    }
}
