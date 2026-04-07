package org.firstinspires.ftc.teamcode.systems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class Intake {

    private DcMotor intake, transfer;
    private Servo gate, pto, pto2;

    public static double shotDelay = 1.2, shotTransferAllowance = 0.4;
    public static double flapActuationTime = 0.8;

    private boolean intakeActivityFlag = false;
    private boolean reverseFlag = false;

    public static double transferPosition = 0.55, lockedPosition = 0.4, ptoEngaged = 0.15, ptoDisengaged = 0;

    public Intake(HardwareMap hardwareMap) {
        intake = hardwareMap.dcMotor.get("intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        transfer = hardwareMap.dcMotor.get("transfer");
        gate = hardwareMap.servo.get("gate");
    }

    public void setActive(boolean flag) {
        intakeActivityFlag = flag;
    }

    public void setReverse(boolean flag) {
        reverseFlag = flag;
    }

    private double transferPower = 1, intakePower = 1;

    public void setTransferPower(double power) {
        transferPower = power;
    }

    public void setIntakePower(double power) {
        intakePower = power;
    }

    public void setTransfer(boolean transfer) {
        if (transfer) {
            this.transfer.setPower(transferPower);
        } else {
            this.transfer.setPower(0);
        }
    }
    public void setTransferSlower(boolean transfer) {
        if (transfer) {
            this.transfer.setPower(0.8);
        } else {
            this.transfer.setPower(0);
        }
    }

    ElapsedTime shotTimer = new ElapsedTime();
    private boolean wasShooting = false;

    int shotCount = 0;

    public void update() {
        if (intakeActivityFlag) {
            intake.setPower(intakePower);
        } else if (!reverseFlag && !intakeActivityFlag) {
            intake.setPower(0);
        } else if (reverseFlag && !intakeActivityFlag){
            intake.setPower(-intakePower);
            transfer.setPower(-intakePower);
        }
    }

    public void setIntake(double power) {
        intake.setPower(power);
    }

    public void setFlap(double pos) {
        gate.setPosition(pos);
    }

    public void setPtoEngaged(boolean engaged) {
        if (engaged) {
            pto.setPosition(0.08);
            pto2.setPosition(0.96);
        } else {
            pto.setPosition(0.2);
            pto2.setPosition(0.85);
        }
    }

}
