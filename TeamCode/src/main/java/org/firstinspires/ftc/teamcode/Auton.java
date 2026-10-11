package org.firstinspires.ftc.teamcode;




import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorEx;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;


import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import java.util.Arrays;
//import static com.qualcomm.hardware.rev.RevHubOrientationOnRobot.xyzOrientation;
import com.qualcomm.robotcore.hardware.IMU;


@Autonomous(name="Auton")
// @Disabled
public class Auton extends LinearOpMode{
    private Limelight3A limelight;
    public static boolean isAuton = true;

    public enum MovePattern {
        moveStart,
        move1,
        waitmove1,
        move2,
        waitmove2,
        move3,
        waitmove3,
        move4,
        waitmove4,
        move5,
        waitmove5,
        done
    };
    MovePattern movePattern = MovePattern.done;

    public enum ArtifactDetector {
        start,
        noartifact,
        purple,
        green
    };
    ArtifactDetector artifactDetector = ArtifactDetector.start;

    public enum IntakeState {
        ready,
        empty,
        hasOne,
        hasTwo,
        hasThree,
        miniStacking,
        miniStacked,
        stacked,
        calculateExitDistance,
        A,
        B,
        C,
        D,
        E,
        // X,
        setTarget,
        waitForCatapult,
        intakeToOuttake,
        presentNextArtifact,
        waitForLaunch,
        done
    };

    IntakeState intakeState = IntakeState.ready;

    public enum WaitState {
        wait,
        done
    };
    WaitState waitState = WaitState.done;


    public enum ProfileState {
        accel,
        slew1,
        slew2,
        decel,
        done
    };
    ProfileState profileState = ProfileState.done;

    public enum CurveState {
        calculatingCoefficients,
        calculatingCommands,
        done
    };
    CurveState curveState = CurveState.done;

    public enum CatapultLaunchState {
        ready,
        waitForArtifactReady,
        servoRelease,
        waitForCatapultReady,
        //servoReturn,
        armBack,
        goToSlack,
        done,
    }
    CatapultLaunchState catapultLaunchState = CatapultLaunchState.waitForArtifactReady;
    // CatapultLaunchState catapultLaunchState = CatapultLaunchState.done;

    int loops = 0;
    boolean in = false;
    boolean pressed = false;
    IMU imu;
    YawPitchRollAngles orientation;

    double newTarget;
    double inverseTarget;


    double thetakp = 0.1; // 0.045
    double xkp = 0.2; // .3
    double ykp = 0.2; // .15
    double xKv = 0.4; //.4
    double yKv = 0.4;
    double thetaKv = 0.05; // velocity constant

    double botProfileVelocity = 0;
    double botProfileAcceleration = 0;


    double unboundedPower;
    int state = 0;

    double thetaCommand = 0;
    double xCommand = 0;
    double yCommand = 0;
    double botProgressCommand = 0;

    double xPositionSlope = 0;
    double yPositionSlope = 0;
    double thetaPositionSlope = 0;

    double xStartPosition = 0;
    double yStartPosition = 0;
    double thetaStartPosition = 0;


    boolean includeTheta = true;
    boolean includex = true;
    boolean includey = true;
    boolean atBase=false;
    double slewSpeed = 1.2;
    int accelTime = 25; // num of loops to take when accelerating
    double halfTargetDistance = 0;


    double xTarget = 0;
    double yTarget = 0;
    double thetaTarget = 90;
    double botTarget = 0;
    double slideCountsPerInch = 82;

    double leftFrontPower = 0;
    double rightFrontPower = 0;

    double leftBackPower = 0;
    double rightBackPower = 0;

    double thetaLeftFrontPower = 0;
    double thetaRightFrontPower = 0;
    double thetaLeftBackPower = 0;
    double thetaRightBackPower = 0;
    double position=0;

    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    private DcMotor hangArm = null;

    double autonDuration = 0;

    private ElapsedTime runtime = new ElapsedTime();

    double frontEncoderInitial = 0;
    double backEncoderInitial = 0;
    double perpInitial = 0;

    double thetaErrorNm1 = 0; //error 1 step before
    double thetaErrorN = 0; // current error
    double thetaVelocity = 0; //difference between current and previous error

    double xErrorNm1 = 0;
    double xErrorN = 0;
    double xVelocity = 0;
    boolean move2done=false;


    double yErrorNm1 = 0;
    double yErrorN = 0;
    double yVelocity = 0;

    double nfrontEncoder = 0;
    double nbackEncoder = 0;
    double nperp = 0;

    double nm1frontEncoder = 0;
    double nm1backEncoder = 0;
    double nm1perp = 0;

    double deltafrontEncoder = 0;
    double deltabackEncoder = 0;
    double deltaperp = 0;

    double botx = 48;
    double boty = 9;

    double deltax = 0;
    double deltay = 0;

    double inchesPerCount = 0.002844488;
    double averagePar = 0;
    double thetaRadians = 0;

    double botDirectionToGo = 0;
    double powerVectorDirection = 0;
    double linearPower = 0;

    double xPower = 0;
    double yPower = 0;

    int delayCounter = 0;
    int moveCounter = 0;
    int pauseCounter = 0;
    int counter = 0;
    int waitCounter=0;

    int halfTime = 0;
    int halfSlewTime = 0;


    double thetaScale = 0.3;
    double powerCompensation = 0.03;

    double perpCompensation = 0;
    double perpRadius = 4; //distance from perp encoder

    double xThird = 0;
    double xSecond = 0;
    double xFirst = 0;
    double xZero = 0;

    double segmentDuration = 50;
    double xInitPosition = 0;
    double xInitVelocity = 0;
    double xInitAcceleration = 0;

    double xFinalPosition = 0;
    double xFinalVelocity = 0;
    double xFinalAcceleration = 0;

    double yThird = 0;
    double ySecond = 0;
    double yFirst = 0;
    double yZero = 0;
    double yInitPosition = 0;
    double yInitVelocity = 0;
    double yInitAcceleration = 0;
    double yFinalPosition = 0;
    double yFinalVelocity = 0;
    double yFinalAcceleration = 0;

    double thetaThird = 0;
    double thetaSecond = 0;
    double thetaFirst = 0;
    double thetaZero = 0;
    double thetaInitPosition = 0;
    double thetaInitVelocity = 0;
    double thetaInitAcceleration = 0;
    double thetaFinalPosition = 0;
    double thetaFinalVelocity = 0;
    double thetaFinalAcceleration = 0;

    double servoWaitCounter = 0;

    double kfactor = 2;



    int PVT_index = -1;
    int PVT_length = 12;

    //PVT Table
    double[] Xp = new double[PVT_length];
    double[] Yp = new double[PVT_length];
    double[] Tp = new double[PVT_length];

    double[] Xv = new double[PVT_length];
    double[] Yv = new double[PVT_length];
    double[] Tv = new double[PVT_length];

    int slackValue = ((-1100* 312)/435);

    int currentIndex = 1;
    int value = 25;
    double[] PVTsegmentDuration = new double[PVT_length];

    private DcMotor arm = null;
    private DcMotor slide = null;
    private Servo clawServo;
    private Servo gripServo;

    int tiltCountsPerDegree = 37;

    String gripperMode= "external";

    String nextGripperMode = "external";

    int testdelay=0;

    int otherTeamWaitCount = 0;
    boolean secondSpec = true;

    double xterm1 = 0;
    double xterm2 =0;
    double yterm1 = 0;
    double yterm2 = 0;
    Servo catapultServo;
    DcMotor catapultMotor;
    TouchSensor test_touch;
    //color reads purple at mid to be 55 red 75 green 105 blue
    //reads green at 50 175 133
    ColorSensor color;
    ColorSensor color2;
    DcMotor motor;
    // DcMotor catapultMotor;
    // Servo servoGate;
    DcMotor transfermotor;





    // Define a variable for our color sensor
    // Servo pushServoOuttake;



    //1 inch and a quarter
    int r = 0;
    int g = 0;
    int p = 0;
    int gr = 0;

    int ballCounter = 0;


    boolean isPurple;
    boolean isGreen;
    boolean catapultReady = true;

    boolean artifactReady = false;

    boolean doneLaunching = false;

    //12959-12705=254
    int GreenSensorCounter = 0;
    int GreenLoops = 5;
    int PurpleSensorCounter = 0;
    int PurpleLoops = 5;
    int desiredArtifact = 1;
    double classifier;

    boolean isMove4=false;

    boolean xlastButton=false;
    boolean ylastButton=false;
    int motorPos=0;
    static final int noArtifact = 0;
    static final int greenArtifact = 1;
    static final int purpleArtifact = 2;
    private DigitalChannel redLED1;
    private DigitalChannel greenLED1;
    private DigitalChannel redLED2;
    private DigitalChannel greenLED2;
    int delay=0;
    int[] artifactOrder = new int[3]; //order as artifacts come in
    int artifactIndex = 0;


    int[] patternOrder = new int[3];//order of the motif
    int patternIndex = 0; //what artifact we want

    double unlockPos = 0.45;
    double lockPos = 0.56; //0.67

    int motorReference = 0;//where you get the third ball motor position
    int miniStackTarget = 75;
    int stackTarget = 500; //1000 distance to make all artifacts be stacked together
    int zeroExitDistance = 0;
    int oneExitDistance = -280;//-260
    int twoExitDistance = -600;//-600

    int[] resequence = new int[3];
    int setValue = 300;

    int launchWait = 0;
    int intakeToOuttakeLoops = 0;
    int offset = 0;

    @Override
    public void runOpMode() {
        if (isAuton){
            movePattern = MovePattern.moveStart;
            intakeState = IntakeState.done;
        }

        xCommand = botx;
        yCommand = boty;
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "leftBackDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBackDrive");
        catapultServo = hardwareMap.get(Servo.class, "servo1");
        // servoGate = hardwareMap.get(Servo.class, "servoGate");
        catapultMotor = hardwareMap.get(DcMotor.class, "motor2");
        test_touch = hardwareMap.get(TouchSensor.class, "touch");
        transfermotor = hardwareMap.get(DcMotor.class, "transferMotor");
        redLED1 = hardwareMap.get(DigitalChannel.class, "red1");
        greenLED1 = hardwareMap.get(DigitalChannel.class, "green1");
        redLED2 = hardwareMap.get(DigitalChannel.class, "red2");
        greenLED2 = hardwareMap.get(DigitalChannel.class, "green2");


        color = hardwareMap.get(ColorSensor.class, "color");
        color2 = hardwareMap.get(ColorSensor.class, "color2");

        motor = hardwareMap.get(DcMotor.class, "intake");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setDirection(DcMotor.Direction.FORWARD);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(7);
        limelight.start();
        // arm = hardwareMap.get(DcMotor.class, "arm");
        // hangArm = hardwareMap.get(DcMotor.class, "hangArm");
        // slide = hardwareMap.get(DcMotor.class, "slide");
        // clawServo = hardwareMap.get(Servo.class, "clawServo");
        // gripServo = hardwareMap.get(Servo.class, "gripServo");


        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);


        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



        // slide.setDirection(DcMotor.Direction.REVERSE);
        // slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // slide.setTargetPosition(0);
        // slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // slide.setPower(0);



        frontEncoderInitial = -leftFrontDrive.getCurrentPosition();
        backEncoderInitial = -leftBackDrive.getCurrentPosition();
        perpInitial = -rightFrontDrive.getCurrentPosition();

        botProfileAcceleration =  slewSpeed/accelTime;
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // catapultServo.setPosition(0.67);


        catapultMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        catapultMotor.setDirection(DcMotor.Direction.FORWARD);
        catapultMotor.setPower(0.2);


        catapultServo.setPosition(lockPos); //locked pos

        // catapultServo.setPosition(0.5); //unlocked pos
        catapultMotor.setTargetPosition((5000* 312)/435);
        catapultMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        while (!test_touch.isPressed() ){
            telemetry.addData("button press?: ", pressed);
            telemetry.addData("target position: ", catapultMotor.getTargetPosition());
            telemetry.update();
        }
        pressed = true;
        telemetry.addData("button press?: ", pressed);
        telemetry.update();
        catapultMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        catapultMotor.setTargetPosition(slackValue);
        catapultMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        LLResult resultinit = limelight.getLatestResult();
        if (resultinit != null && resultinit.isValid()) {
            for (FiducialResult fiducial : resultinit.getFiducialResults()) {
                int id = fiducial.getFiducialId();  // <-- your AprilTag ID
                telemetry.addData("ID", id);
                telemetry.update();
            }
        }
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            // telemetry.setAutoClear(false);

            // if (gamepad1.y){
            //     catapultReady=true;
            // }
            // if(gamepad1.y && !ylastButton){
            //     catapultReady=true;
            // }
            // ylastButton=gamepad1.y;

            // if(gamepad1.x && !xlastButton){
            //     artifactReady=true;
            // }
            // xlastButton=gamepad1.x;
            if (!isAuton){
                xCommand += Math.pow(gamepad1.left_stick_x, 3)*1.8;
                yCommand -= Math.pow(gamepad1.left_stick_y, 3)*1.8;

                if (gamepad1.right_stick_x!=0){
                    thetaCommand += Math.pow(gamepad1.right_stick_x, 3)*2.8;
                }
                if(gamepad2.dpad_right){
                    intakeState = IntakeState.miniStacking;
                }
            }

            // if(gamepad1.dpad_up){
            //     catapultServo.setPosition(unlockPos);
            // }
            // if(gamepad1.dpad_down){
            //     catapultServo.setPosition(lockPos);//0.67
            // }

            // redLED1.setMode(DigitalChannel.Mode.OUTPUT);
            // greenLED1.setMode(DigitalChannel.Mode.OUTPUT);
            // redLED2.setMode(DigitalChannel.Mode.OUTPUT);
            // greenLED2.setMode(DigitalChannel.Mode.OUTPUT);

            // redLED1.setState(false);
            // clearPVTtable();

            leftFrontPower = 0;
            rightFrontPower = 0;
            leftBackPower = 0;
            rightBackPower = 0;
            // motor.setPower(1);
            catapultMotor.setPower(1);


            double max;
            double min;
            // telemetry.addData("movepattern state", movePattern);
            // telemetry.addData("curvestate ", curveState);
            // telemetry.addData("servo wait counter ", servoWaitCounter);
            // telemetry.addData("perp", perpPosition());

            // telemetry.addData("botx", botx);
            // telemetry.addData("boty", boty);
            // telemetry.addData("angle", thetaPos());
            // telemetry.addData("thetacommand", thetaCommand);


            // telemetry.addData("xError", xError());
            // telemetry.addData("yError", yError());
            // telemetry.addData("theta error: ", thetaError());


            // telemetry.addData("curve state: ", curveState);

            // telemetry.addData("move state: ", movePattern);
            // telemetry.addData("pvt segment duration", PVTsegmentDuration[currentIndex]);

            // telemetry.addData("move pattern state", movePattern);
            // telemetry.addData("hang arm", hangArm.getTargetPosition());
            // telemetry.addData("slide", slide.getTargetPosition());
            // telemetry.addData("arm target", arm.getTargetPosition());
            // telemetry.addData("arm current", arm.getCurrentPosition());
            // telemetry.addData("curve state", curveState);
            // telemetry.addData("wait state", waitState);
            // telemetry.addData("servo wait counter", servoWaitCounter);
            // telemetry.addData("time", autonDuration);
            // telemetry.addData("front encoder: ", frontEncoderPosition());
            // telemetry.addData("back encoder: ", backEncoderPosition());
            // telemetry.addData("motor target: ", motor.getTargetPosition());
            // telemetry.addData("xterm1 ", xterm1);
            // telemetry.addData("xterm2 ", xterm2);
            // telemetry.addData("yterm1 ", yterm1);
            // telemetry.addData("yterm2 ", yterm2);

            // telemetry.addData("front encoder")

//front 22, -1726 back 1070, -665, angle 40


            // telemetry.update();
            int r = color.red();
            int g = color.green();
            int b = color.blue();
            int r2 = color2.red();
            int g2= color2.green();
            int b2= color2.blue();
            LLResult result = limelight.getLatestResult();





            // Decide what color we see
            String detectedColor = "Unknown";
            String detectedColor2 = "Unknown";
            //1 inch 60 red 300 green 250 blue
            //0.5 inch 100 red 500 green 300 blue
            // 600 red 2500 green 2000 blue
            //1300 red 5000 green 3700 blue


            //purp 1 inch 150 red 150 green 250 blue
            //0.5 inch 400 red 400 green 750 blue
            //0.25 inch 1800 red 1900 green 3400 blue
            //2400 2500 4300
            // telemetry.addData("Red", r);
            // telemetry.addData("Green", g);
            // telemetry.addData("Blue", b);
            // telemetry.addData("Detected Color", detectedColor);
            // telemetry.addData("Detected Color2", detectedColor2);

            if (result != null) {
                if (result.isValid()) {
                    Pose3D botpose = result.getBotpose();
                    // telemetry.addData("tx", result.getTx());
                    // telemetry.addData("ty", result.getTy());
                    if (result != null && result.isValid()) {
                        for (FiducialResult fiducial : result.getFiducialResults()) {
                            int id = fiducial.getFiducialId();  // <-- your AprilTag ID
                            telemetry.addData("ID", id);
                            if (id == 21){
                                patternOrder[0] = 1;
                                patternOrder[1] = 2;
                                patternOrder[2] = 2;
                            } else if (id == 22){
                                patternOrder[0] = 2;
                                patternOrder[1] = 1;
                                patternOrder[2] = 2;
                            } else if (id == 23){
                                patternOrder[0] = 2;
                                patternOrder[1] = 2;
                                patternOrder[2] = 1;
                            } else if (id==20){
                                classifier=(result.getTx()+15);
                            }
                            telemetry.update();
                        }


                    }


                }
            }
            //color reads purple at mid to be 55 red 75 green 105 blue
            //purple hole 60 79 117
            //reads green at 50 175 133
            //green 35 122 90
            //33 100 77
            // Simple thresholds — you’ll likely want to tune these by testing
            if (g > 2*r && g > b ) {
                detectedColor = "Green";
                GreenSensorCounter ++;
            } else {
                GreenSensorCounter--;
            }

            if (g2 > 2*r2 && g2 > b2) {
                detectedColor2 = "Green";

            }


            if (GreenSensorCounter > GreenLoops) {
                GreenSensorCounter = GreenLoops;
            }
            if (GreenSensorCounter < 0) {
                GreenSensorCounter = 0;
            }
            if (GreenSensorCounter == 2) {
                isGreen = true;
            }
            if (GreenSensorCounter == 0) {
                isGreen = false;
            }
            //color reads purple at mid to be 55 red 75 green 105 blue
            //purple hole 60 79 117
            if (b > g && b > r*1.2) {
                // Purple is a mix of red + blue, with less green
                detectedColor = "Purple";
                PurpleSensorCounter++;
            } else {
                PurpleSensorCounter--;
            }

            if (b2 > g2 && b2 > r2*1.2) {
                // Purple is a mix of red + blue, with less green
                detectedColor2 = "Purple";
            }


            if (PurpleSensorCounter > PurpleLoops) {
                PurpleSensorCounter = PurpleLoops;
            }
            if (PurpleSensorCounter < 0) {
                PurpleSensorCounter = 0;
            }
            if (PurpleSensorCounter == 2) {
                isPurple = true;
            }
            if (PurpleSensorCounter == 0) {
                isPurple = false;
            }





            switch (artifactDetector){
                case start:
                    if (detectedColor.equals("Unknown")) {
                        artifactDetector = ArtifactDetector.noartifact;
                    }
                    break;


                case noartifact:
                    if (isPurple && artifactIndex < 3) {
                        p++;
                        artifactOrder[artifactIndex++] = purpleArtifact;
                        artifactDetector = ArtifactDetector.purple;
                    }
                    if (isGreen && artifactIndex < 3) {
                        gr++;
                        artifactOrder[artifactIndex++] = greenArtifact;
                        artifactDetector = ArtifactDetector.green;
                    }

                    if((detectedColor2 == "green" && g==1) || (detectedColor2 == "purple" && p==2)){
                        // servoGate.setPosition(1);
                    }
                    break;


                case purple:
                    if (isGreen && artifactIndex < 3) {
                        gr++;
                        artifactOrder[artifactIndex++] = greenArtifact;
                        artifactDetector = ArtifactDetector.green;
                    } else if (!isPurple) {
                        artifactDetector = ArtifactDetector.noartifact;
                    }
                    break;


                case green:
                    if (isPurple && artifactIndex < 3) {
                        p++;
                        artifactOrder[artifactIndex++] = purpleArtifact;
                        artifactDetector = ArtifactDetector.purple;
                    } else if (!isGreen) {
                        artifactDetector = ArtifactDetector.noartifact;
                    }
                    break;
            }





            switch (intakeState) {
                case ready:
                    if (gamepad2.a) {
                        intakeState = IntakeState.empty;
                    }
                    break;

                case empty:
                    motor.setPower(1);

                    if (gr+p == 1) {
                        intakeState = IntakeState.hasOne;
                    }
                    break;

                case hasOne:
                    // if (gr > 0){

                    // }

                    if (gr+p == 2) {
                        intakeState = IntakeState.hasTwo;
                    }
                    break;

                case hasTwo:

                    if (gr+p == 3) {
                        motorReference = motor.getCurrentPosition();
                        intakeState = IntakeState.hasThree;
                    }
                    break;

                case hasThree:
                    // servoGate.setPosition(1);
                    if (motor.getCurrentPosition() >= (motorReference + miniStackTarget)) {
                        motor.setPower(0);
                        motorReference = motor.getCurrentPosition();
                        intakeState = IntakeState.miniStacking;
                    }
                    break;

                case miniStacking:
                    motorReference = motor.getCurrentPosition();
                    if(gamepad2.dpad_left && !isAuton){
                        motorPos += (40);
                        motor.setTargetPosition(motor.getCurrentPosition()+motorPos);
                        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        motor.setPower(1);
                    }else if (!isAuton){
                        motor.setPower(0);
                    }


                    if (gamepad2.y || isAuton) {
                        motor.setPower(1);
                        intakeState = IntakeState.miniStacked;
                        // if (result != null) {
                        //     if (result.isValid()) {

                    }
                    //     }
                    // }
                    break;

                case miniStacked:


                    if (motor.getCurrentPosition() >= (motorReference + stackTarget)) {
                        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        intakeState = IntakeState.stacked;
                    }
                    break;

                case stacked:
                    if (gamepad2.b ) {
                        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        // LLResult result = limelight.getLatestResult();
                        if (result != null && result.isValid()) {
                            for (FiducialResult fiducial : result.getFiducialResults()) {
                                int id = fiducial.getFiducialId();  // <-- your AprilTag ID
                                // telemetry.addData("ID", id);
                                if (id == 20){
                                    thetaCommand+=(classifier*-1);
                                    // clearPVTtable();
                                    // appendPVTtable(botx, boty, (result.getTx()*-1)+thetaPos(), 0, 0, 0, 20);
                                    // movePath();
                                }
                            }
                            telemetry.update();
                        }
                        intakeState = IntakeState.calculateExitDistance;
                    }

                    if (gamepad2.x || (isAuton && atBase)){
                        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        // motor.setPower(0);
                        // motor.setPower(1);
                        // LLResult result = limelight.getLatestResult();
                        if (result != null && result.isValid()) {
                            for (FiducialResult fiducial : result.getFiducialResults()) {
                                int id = fiducial.getFiducialId();  // <-- your AprilTag ID
                                // telemetry.addData("ID", id);
                                if (id == 20){
                                    // thetaCommand+=(classifier*-1)-10;
                                    // clearPVTtable();
                                    // appendPVTtable(botx, boty, (result.getTx()*-1)+thetaPos(), 0, 0, 0, 20);
                                    // movePath();
                                }
                            }
                            telemetry.update();
                        }
                        // motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        intakeState = IntakeState.B;
                    }
                    break;

                //  case calculateExitDistance:
                // if (patternOrder[patternIndex] == artifactOrder[2]) {
                //     motor.setTargetPosition(zeroExitDistance);
                //     artifactOrder[2] = 0;
                //     intakeState = IntakeState.waitForCatapult;
                // } else if (patternOrder[patternIndex] == artifactOrder[1]) {
                //     motor.setTargetPosition(oneExitDistance);
                //     intakeState = IntakeState.waitForCatapult;
                //     artifactOrder[1] = 0;
                // } else if (patternOrder[patternIndex] == artifactOrder[0]) {
                //     motor.setTargetPosition(twoExitDistance);
                //     intakeState = IntakeState.waitForCatapult;
                //     artifactOrder[0] = 0;
                // }
                // motor.setPower(1);
                // motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                case calculateExitDistance:
                    motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    offset = 0;
                    if (Arrays.equals(patternOrder, new int[]{2,2,1})) {

                        if (Arrays.equals(artifactOrder, new int[]{2,2,1})) {
                            // if (patternIndex==2){
                            //     offset=200;
                            // }
                            intakeState = IntakeState.E;
                        }
                        else if (Arrays.equals(artifactOrder, new int[]{2,1,2})) {
                            if (patternIndex==2){
                                offset=100;//usedto be 220
                            }
                            intakeState = IntakeState.A;
                        }
                        else if (Arrays.equals(artifactOrder, new int[]{1,2,2})) {
                            intakeState = IntakeState.B;
                        }

                    }
                    else if (Arrays.equals(patternOrder, new int[]{2,1,2})) {

                        if (Arrays.equals(artifactOrder, new int[]{2,2,1})) {
                            intakeState = IntakeState.E; //d
                        }
                        else if (Arrays.equals(artifactOrder, new int[]{2,1,2})) {
                            intakeState = IntakeState.B;
                        }
                        else if (Arrays.equals(artifactOrder, new int[]{1,2,2})) {
                            if (patternIndex==2){
                                offset=100;
                            }
                            intakeState = IntakeState.A;
                        }

                    }
                    else if (Arrays.equals(patternOrder, new int[]{1,2,2})) {

                        if (Arrays.equals(artifactOrder, new int[]{2,2,1})) {
                            intakeState = IntakeState.B;
                        }
                        else if (Arrays.equals(artifactOrder, new int[]{2,1,2})) {
                            intakeState = IntakeState.E;
                        }
                        else if (Arrays.equals(artifactOrder, new int[]{1,2,2})) {
                            // if (patternIndex==1){
                            //     offset=120;
                            // }
                            // if (patternIndex==2){
                            //     offset=50;
                            // }
                            intakeState = IntakeState.C;
                        }
                    }
                    break;


                case E:
                    resequence = new int[]{-setValue, 100, -2*setValue};
                    intakeState = IntakeState.setTarget;
                    break;

                case A:
                    resequence = new int[]{-setValue, 0, -2*setValue};
                    intakeState = IntakeState.setTarget;
                    break;

                case B:
                    // motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    resequence = new int[]{0, -setValue+35, -2*setValue+35};
                    intakeState = IntakeState.setTarget;
                    break;

                case C:
                    resequence = new int[]{-2 * setValue, -setValue+120, 50};
                    intakeState = IntakeState.setTarget;
                    break;

                case D:
                    resequence = new int[]{-setValue+70, -2*setValue+50,300};
                    intakeState = IntakeState.setTarget;
                    break;

                // case X:
                //     resequence = new int[]{0, -setValue, 2 * setValue};
                //     intakeState = IntakeState.setTarget;
                //     break;

                case setTarget:
                    position = resequence[patternIndex];
                    motor.setTargetPosition((int)position);
                    motor.setPower(1);
                    motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    intakeState = IntakeState.waitForCatapult;
                    break;


                case waitForCatapult:
                    if(catapultReady){
                        intakeToOuttakeLoops = 0;
                        intakeState = IntakeState.intakeToOuttake;
                        catapultReady=false;
                    }
                    break;

                case intakeToOuttake:

                    if (Math.abs(motor.getCurrentPosition()-motor.getTargetPosition()) <= 20){
                        transfermotor.setPower(-1);
                        intakeToOuttakeLoops++;
                        if (intakeToOuttakeLoops > 25){
                            transfermotor.setPower(0);

                            artifactReady=true;

                            intakeState = IntakeState.presentNextArtifact;
                        }
                    }
                    break;

                case presentNextArtifact:
                    if (patternIndex>=2){
                        intakeState = IntakeState.done;
                    } else {
                        patternIndex++;
                        intakeState = IntakeState.setTarget;
                    }
                    break;

                // case waitForLaunch:
                // if (patternIndex == 3 && Math.abs(catapultMotor.getCurrentPosition()-catapultMotor.getTargetPosition()) < 30){
                //     intakeState = IntakeState.done;
                // } else {
                // intakeState = IntakeState.calculateExitDistance;
                // }

                // break;
                // case presentNextArtifact:
                //         if (patternIndex >= 2 && Math.abs(catapultMotor.getCurrentPosition()-catapultMotor.getTargetPosition()) < 30){
                //             intakeState = IntakeState.done;
                //         } else {
                //             patternIndex++;
                //             catapultLaunchState = catapultLaunchState.ready;
                //             intakeState = IntakeState.calculateExitDistance;
                //         }




                //     break;

                // case waitForLaunch:
                //     if (catapultLaunchState == catapultLaunchState.done) {
                //         if (patternIndex == 3 && Math.abs(catapultMotor.getCurrentPosition()-catapultMotor.getTargetPosition()) < 30){
                //             intakeState = IntakeState.done;
                //         } else {
                //             intakeState = IntakeState.calculateExitDistance;
                //         }
                //     }
                //     break;

                // case waitForLaunch:
                //     if (patternIndex == 3 && Math.abs(catapultMotor.getCurrentPosition()-catapultMotor.getTargetPosition()) < 30){
                //             intakeState = IntakeState.done;
                //         } else {
                //             intakeState = IntakeState.calculateExitDistance;
                //         }
                //     break;

                case done:
                    // pushServoOuttake.setPosition(0);
                    // catapultMotor.setTargetPosition(0);
                    motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    motor.setPower(0);
                    artifactIndex = 0;
                    patternIndex=0;
                    artifactOrder[0] = 0;
                    artifactOrder[1] = 0;
                    artifactOrder[2] = 0;
                    gr = 0;
                    p = 0;
                    if (!isAuton){
                        intakeState = IntakeState.ready;
                        artifactDetector = ArtifactDetector.start;

                    }
                    // intakeState = IntakeState.launch;
                    break;


            }

            switch (movePattern) {
                case moveStart:
                    movePattern = MovePattern.move1;
                    artifactOrder[0] = 2;
                    artifactOrder[1] = 1;
                    artifactOrder[2] = 2;

                    break;

                case move1:

                    clearPVTtable();
                    appendPVTtable(60, 18, (classifier), 0, 0, 0, 20*kfactor);//30.75
                    movePath();
                    intakeState = IntakeState.stacked;
                    delay=0;
                    movePattern = MovePattern.waitmove1;

                    break;

                case waitmove1:

                    atBase=true;
                    // if (delay>=25 && intakeState == IntakeState.done && curveState==CurveState.done){
                    if (doneLaunching){
                        atBase=false;
                        doneLaunching = false;
                        movePattern = MovePattern.move2;
                    }


                    break;

                case move2:
                    // inMove2=true;
                    move2done=true;
                    clearPVTtable();
                    appendPVTtable(41, 36, 115, 0, 0, 0, 35*kfactor);//30.75
                    appendPVTtable(15, 44, 120, 0, 0, 0, 30*kfactor);//30.75
                    movePath();
                    intakeState = IntakeState.empty;
                    artifactDetector = ArtifactDetector.start;
                    movePattern = MovePattern.waitmove2;



                    break;

                case waitmove2:


                    if (curveState == CurveState.done){

                        intakeState=IntakeState.stacked;
                        movePattern = MovePattern.move3;
                    }

                    break;


                case move3:
                    clearPVTtable();
                    appendPVTtable(60, 21, (classifier), 0, 0, 0, 20*kfactor);//30.75
                    movePath();
                    // clearPVTtable();
                    // appendPVTtable(48, 50, 135, 0, 0, 0, 50*kfactor);//30.75
                    // appendPVTtable(28, 50, 135, 0, 0, 0, 50*kfactor);//30.75
                    // appendPVTtable(60, 22, 20, 0, 0, 0, 50*kfactor);//30.75

                    // movePath();
                    movePattern = MovePattern.waitmove3;

                    break;

                case waitmove3:
                    if (curveState == CurveState.done){
                        atBase=true;
                    }
                    if (doneLaunching) {
                        doneLaunching=false;
                        movePattern = MovePattern.move4;
                    }

                    break;

                // case waitwaitmove3:
                //     if (waitState == WaitState.done){
                //         movePattern = MovePattern.move4;
                //     }
                // break;

                case move4:
                    // isMove4 = true;
                    clearPVTtable();
                    atBase=false;
                    appendPVTtable(52, 65.58, 120.08, 0, 0, 0, 30*kfactor);//30.75
                    appendPVTtable(26, 69.4, 123.08, 0, 0, 0, 30*kfactor);//30.75
                    movePath();
                    intakeState = IntakeState.empty;
                    artifactDetector = ArtifactDetector.start;
                    movePattern = MovePattern.waitmove4;

                    break;

                case waitmove4:


                    if (curveState == CurveState.done){

                        intakeState=IntakeState.stacked;
                        movePattern = MovePattern.done;
                    }

                    break;

                case move5:
                    clearPVTtable();
                    appendPVTtable(60, 21, (classifier), 0, 0, 0, 20*kfactor);//30.75
                    movePath();
                    // clearPVTtable();
                    // appendPVTtable(48, 50, 135, 0, 0, 0, 50*kfactor);//30.75
                    // appendPVTtable(28, 50, 135, 0, 0, 0, 50*kfactor);//30.75
                    // appendPVTtable(60, 22, 20, 0, 0, 0, 50*kfactor);//30.75

                    // movePath();
                    movePattern = MovePattern.waitmove5;

                    break;

                case waitmove5:
                    if (curveState == CurveState.done){
                        atBase=true;
                        movePattern = MovePattern.done;
                    }

                    break;

                case done:

                    break;

                default:
                    movePattern = MovePattern.done;
            }
            switch (waitState) {
                case wait:
                    waitCounter++;
                    if (waitCounter>=100){
                        waitState = WaitState.done;
                    }
                    break;

                case done:
                    break;
            }

            switch(catapultLaunchState){

                case ready:
                    catapultLaunchState = CatapultLaunchState.waitForCatapultReady;
                    break;


                // case servoReturn:
                //     loops = 0;
                //     catapultLaunchState = CatapultLaunchState.armBack;

                //     break;
                case waitForCatapultReady:
                    // if (!test_touch.isPressed() ){
                    //     motor.setPower(1);
                    // } else{
                    //     motor.setPower(0);
                    //     catapultLaunchState = CatapultLaunchState.armBack;
                    // }
                    loops++;

                    if (loops >= 8){
                        catapultMotor.setTargetPosition((20* 312)/435);
                        catapultServo.setPosition(lockPos);

                    }
                    break;

                case armBack:
                    catapultReady=true;
                    if ((Math.abs(catapultMotor.getCurrentPosition() - catapultMotor.getTargetPosition()) <= 36)||(test_touch.isPressed())){
                        // testdelay=0;
                        catapultLaunchState = CatapultLaunchState.goToSlack;
                    }
                    break;

                case goToSlack:
                    //   testdelay++;
                    catapultMotor.setTargetPosition(slackValue);
                    if (Math.abs(catapultMotor.getCurrentPosition() - catapultMotor.getTargetPosition()) <= 10){
                        // if(testdelay>=10){
                        catapultLaunchState =  CatapultLaunchState.waitForArtifactReady;
                        // }
                    }
                    break;

                case waitForArtifactReady:
                    if(artifactReady && (curveState.equals(CurveState.done))){
                        catapultLaunchState = CatapultLaunchState.servoRelease;
                    }
                    break;

                case servoRelease:
                    catapultServo.setPosition(unlockPos);
                    ballCounter++;
                    if (ballCounter == 3){
                        doneLaunching = true;
                        ballCounter =0;
                    }
                    artifactReady=false;
                    loops = 0;
                    catapultLaunchState = CatapultLaunchState.ready;
                    break;


                case done:
                    break;
            }




            switch (profileState) {
                case accel:
                    if(botProgressCommand>=halfTargetDistance){ // check if we passed half the distance, triangle case
                        profileState = ProfileState.decel;
                    }
                    else if(botProfileVelocity<slewSpeed){ // keep accelerating
                        botProfileVelocity+=botProfileAcceleration;
                        // telemetry.addData("in state ", moveCounter);
                    }
                    else { // go to slew 1, stop accelerating
                        botProfileVelocity = slewSpeed;
                        profileState = ProfileState.slew1;
                    }
                    break;

                case slew1:
                    if(botProgressCommand>=halfTargetDistance){
                        halfTime = moveCounter;
                        profileState = ProfileState.slew2;
                        halfSlewTime = halfTime - accelTime;
                    }
                    break;

                case slew2:
                    if(moveCounter==halfSlewTime+halfTime){
                        profileState = ProfileState.decel;
                    }
                    break;

                case decel:
                    botProfileVelocity -= botProfileAcceleration;
                    if (botProfileVelocity < 0.001){
                        botProfileVelocity = 0.001;
                    }
                    if(botProgressCommand >= botTarget){
                        botProgressCommand = botTarget;
                        profileState = ProfileState.done;
                    }
                    break;

                case done:
                    botProfileVelocity = 0;
                    break;

                default:
                    profileState = ProfileState.done;
            }

            switch (curveState){
                case calculatingCoefficients:
                    calculateCoefficients(currentIndex);
                    curveState = CurveState.calculatingCommands;
                    break;

                case calculatingCommands:
                    if ((moveCounter > PVTsegmentDuration[currentIndex])){
                        currentIndex += 1;
                        if (currentIndex > PVT_index){
                            curveState = CurveState.done;
                        } else {
                            calculateCoefficients(currentIndex);


                        }
                    }

                    // && (Math.abs(xErrorN)<1) && (Math.abs(yErrorN)<1) && (Math.abs(thetaErrorN)<3)
                    xCommand = ((xThird*Math.pow(moveCounter,3)) + (xSecond*Math.pow(moveCounter,2)) + (xFirst*moveCounter) + (xZero));
                    yCommand = ((yThird*Math.pow(moveCounter,3)) + (ySecond*Math.pow(moveCounter,2)) + (yFirst*moveCounter) + (yZero));
                    thetaCommand = ((thetaThird*Math.pow(moveCounter,3)) + (thetaSecond*Math.pow(moveCounter,2)) + (thetaFirst*moveCounter) + (thetaZero));







                    break;

                case done:

                    break;

                default:
                    curveState = CurveState.done;
            }



            // yCommand = (yTarget-yStartPosition)*(botProgressCommand/botTarget)+yStartPosition;
            // thetaCommand = (thetaTarget-thetaStartPosition)*(botProgressCommand/botTarget)+thetaStartPosition;

            if (botProgressCommand < botTarget){
                botProgressCommand += botProfileVelocity;
            }

            thetaErrorN = thetaError();
            thetaVelocity = thetaErrorN - thetaErrorNm1;

            thetaLeftFrontPower = -(thetaErrorN*thetakp + thetaVelocity*thetaKv);
            thetaRightFrontPower =(thetaErrorN*thetakp + thetaVelocity*thetaKv);
            thetaLeftBackPower = -(thetaErrorN*thetakp + thetaVelocity*thetaKv);
            thetaRightBackPower =(thetaErrorN*thetakp + thetaVelocity*thetaKv);

            thetaErrorNm1 = thetaErrorN;

            xErrorN = xError();
            xVelocity = xErrorN - xErrorNm1; // how much the error changed from current cycle to previous cycle.
            xPower = xErrorN*xkp + xVelocity * xKv;

            yErrorN = yError();
            yVelocity = yErrorN - yErrorNm1;
            yPower = yErrorN*ykp + yVelocity * yKv;

            xErrorNm1 = xErrorN;
            yErrorNm1 = yErrorN;

            // Theta is angle of robot on the field
            // Error is the direction the bot should travel to from theta=0
            // botDirectionToGo is the direction the bot should travel to from its current theta
            linearPower = Math.sqrt(xPower*xPower + yPower*yPower);
            powerVectorDirection = (Math.atan2(yPower,xPower)*(180/Math.PI)) -90;
            botDirectionToGo = powerVectorDirection - thetaPos();

            double leftFrontPower  = hysteresisPower(Math.cos((botDirectionToGo +45)*(Math.PI/180))* linearPower + thetaLeftFrontPower);
            double rightFrontPower = hysteresisPower(Math.cos((botDirectionToGo -45)*(Math.PI/180))* linearPower + thetaRightFrontPower);
            double leftBackPower   = hysteresisPower(Math.cos((botDirectionToGo -45)*(Math.PI/180))* linearPower + thetaLeftBackPower);
            double rightBackPower  = hysteresisPower(Math.cos((botDirectionToGo +45)*(Math.PI/180))* linearPower + thetaRightBackPower);

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }
            //THIS MINUS SIGN EXISTS BECAUSE IF CHANGING DIRECTION AT START, messes up positive power being forward
            leftFrontDrive.setPower(-leftFrontPower);
            rightFrontDrive.setPower(-rightFrontPower);
            leftBackDrive.setPower(-leftBackPower);
            rightBackDrive.setPower(-rightBackPower);

            // telemetry.addData("lfpower", -leftFrontPower);
            // telemetry.addData("rfpower", -rightFrontPower);
            // telemetry.addData("lbpower", -leftBackPower);
            // telemetry.addData("rbpower", -rightBackPower);
            // telemetry.addData("botdirectiontogo ", botDirectionToGo);
            // telemetry.addData("waitCounter", waitCounter);

            // telemetry.addData("yErrorN", yErrorN);
            // telemetry.addData("xErrorN", xErrorN);
            // telemetry.addData("Red", r);
            // telemetry.addData("Green", g);
            // telemetry.addData("Blue", b);
            // telemetry.addData("Detected Color", detectedColor);
            // telemetry.addData("Detected Color2", detectedColor2);
            // telemetry.addData("theta Error", thetaError());
            // telemetry.addData("tx", (result.getTx()*-1));


            // telemetry.addData("purple counted", p);
            // telemetry.addData("green counted", gr);
            // telemetry.addData("motor2 current pos", catapultMotor.getCurrentPosition());

            // telemetry.addData("catapult ready", catapultReady);
            // telemetry.addData("artifact ready", artifactReady);
            // telemetry.addData("intake state", intakeState);
            // telemetry.addData("pattern index", patternIndex);
            telemetry.addData("intake state", intakeState);
            // telemetry.addData("artifact detector state", artifactDetector);
            // telemetry.addData("motor reference", motorReference);
            // telemetry.addData("motor target pos", motor.getTargetPosition());
            // telemetry.addData("intakemotor current pos", motor.getCurrentPosition());
            // telemetry.addData("Artifact Order", Arrays.toString(artifactOrder));
            // telemetry.addData("pattern order: ", Arrays.toString(patternOrder));
            // telemetry.addData("miniStacked Condition: ", (motor.getCurrentPosition() >= (motorReference + stackTarget)));
            // telemetry.addData("classifier:", classifier);
            telemetry.addData("delay>=25", delay >= 25);
            telemetry.addData("intake done", intakeState == IntakeState.done);
            telemetry.addData("curve done", curveState == CurveState.done);
            // telemetry.addData("move1done", move1done);
            telemetry.addData("movePattern case:", movePattern);
            // telemetry.addData("delay: ", delay);
            // telemetry.addData("current index", currentIndex);
            telemetry.addData("move2done", move2done);
            telemetry.addData("artifact ready: ", artifactReady);
            telemetry.addData("catapult state: ", catapultLaunchState);
            telemetry.addData("curveState done: ", curveState.equals(CurveState.done));

            // telemetry.addData("ID", id);

            telemetry.addData("classifier", classifier);

            // telemetry.a
            // telemetry.addData("pvt index:", PVT_index);
            telemetry.update();



            moveCounter += 1;
            counter +=1;

            perpCompensation = -((perpRadius*2*Math.PI)*(thetaPos()/360))/inchesPerCount;

            nfrontEncoder = frontEncoderPosition();
            nbackEncoder = backEncoderPosition();
            nperp = perpPosition()-perpCompensation;

            deltafrontEncoder = nfrontEncoder - nm1frontEncoder;
            deltabackEncoder = nbackEncoder - nm1backEncoder;
            deltaperp = nperp - nm1perp;

            nm1frontEncoder = nfrontEncoder;
            nm1backEncoder = nbackEncoder;
            nm1perp = nperp;

            averagePar = (deltabackEncoder+deltafrontEncoder)/2;
            thetaRadians = thetaPos()*(Math.PI/180);

            xterm1 = -averagePar*Math.cos(thetaRadians);
            yterm1 = -averagePar*Math.sin(thetaRadians);
            xterm2 = deltaperp*Math.sin(thetaRadians);
            yterm2 = -deltaperp*Math.cos(thetaRadians);

            deltax = -averagePar*Math.cos(thetaRadians)  +  deltaperp*Math.sin(thetaRadians);
            deltay = -averagePar*Math.sin(thetaRadians) - deltaperp*Math.cos(thetaRadians);

            botx += deltax*inchesPerCount;
            boty += deltay*inchesPerCount;






            // telemetry.update();
            // if(value > 20 ){
            //     throw new RuntimeException("value is not high");

            // }

        }
    }

    public double frontEncoderPosition(){
        return -leftFrontDrive.getCurrentPosition() - frontEncoderInitial;
    }

    public double backEncoderPosition(){
        return leftBackDrive.getCurrentPosition() - backEncoderInitial;
    }

    public double perpPosition(){
        return -rightFrontDrive.getCurrentPosition() - perpInitial;
    }

    public void moveTo (double x, double y, double theta){
        // yTarget = y;
        // thetaTarget = theta;
        // yStartPosition = yCommand;
        // thetaStartPosition = thetaCommand;
        botTarget = 0;
        botProfileVelocity = 0;
        botProgressCommand = 0;
        halfTargetDistance = botTarget/2;
        xFinalPosition = x;
        yFinalPosition = y;
        thetaFinalPosition = theta;




        // xZero = xInitPosition;
        // xFirst = xInitVelocity;
        // xThird = (2*(xFinalPosition - xInitPosition - xInitVelocity*segmentDuration)-(xFinalVelocity-xInitVelocity)*Math.pow(segmentDuration,2))/Math.pow(segmentDuration,3);
        // xSecond = (xFinalVelocity - xInitVelocity - (3*xThird*Math.pow(segmentDuration,2)))/(2*segmentDuration);

        // yZero = yInitPosition;
        // yFirst = yInitVelocity;
        // yThird = (2*(yFinalPosition - yInitPosition - yInitVelocity*segmentDuration)-(yFinalVelocity-yInitVelocity)*Math.pow(segmentDuration,2))/Math.pow(segmentDuration,3);
        // ySecond = (yFinalVelocity - yInitVelocity - (3*yThird*Math.pow(segmentDuration,2)))/(2*segmentDuration);

        // thetaZero = thetaInitPosition;
        // thetaFirst = thetaInitVelocity;
        // thetaThird = (2*(thetaFinalPosition - thetaInitPosition - thetaInitVelocity*segmentDuration)-(thetaFinalVelocity-thetaInitVelocity)*Math.pow(segmentDuration,2))/Math.pow(segmentDuration,3);
        // thetaSecond = (thetaFinalVelocity - thetaInitVelocity - (3*thetaThird*Math.pow(segmentDuration,2)))/(2*segmentDuration);



        moveCounter = 0;
        profileState = ProfileState.accel;
    }

    public double boundedPower (double unboundedPower) {
        if (unboundedPower>1){
            return 1;
        }
        else if (unboundedPower<-1){
            return -1;
        }
        else {
            return unboundedPower;
        }
    }

    public double thetaPos(){
        return (frontEncoderPosition() - backEncoderPosition()) * 0.0138158; // 90/8000 (0.01125) old robot
        //3600 = (260572)*x
        //x = 0.0138158 new robot
    }

    public double thetaError (){
        return thetaCommand-thetaPos();
    }

    public double xError(){
        return xCommand-botx; // -0.5 - 0 → error is -0.5
    }

    public double yError(){
        return yCommand-boty;
    }

    public boolean botHasReachedTarget(){
        if (Math.abs(botTarget - botProgressCommand) >= 0.001){
            return false;
        }
        return true;
    }

    public boolean botHasSettled(){
        if (!botHasReachedTarget()){
            delayCounter = 10;
            return false;
        }
        if (delayCounter > 0){
            delayCounter -= 1;
            return false;
        }
        return true;
    }

    public double hysteresisPower(double powerIn){
        if (powerIn > 0){
            return powerIn+powerCompensation;
        }
        else if (powerIn < 0){
            return powerIn-powerCompensation;
        }
        else {
            return 0;
        }
    }

    public void clearPVTtable(){
        PVT_index = -1;

        appendPVTtable(botx, boty, thetaPos(), 0, 0, 0, 0);
    }

    public void appendPVTtable(double Xp_p, double Yp_p, double Tp_p, double Xv_p, double Yv_p, double Tv_p, double PVTsegmentDuration_p){
        PVT_index ++;
        if (PVT_index > PVT_length-1){
            telemetry.addData("PVT Table index out of bounds!!", "):<");
        }
        else {
            Xp[PVT_index] = Xp_p;
            Yp[PVT_index] = Yp_p;
            Tp[PVT_index] = Tp_p;

            Xv[PVT_index] = Xv_p;
            Yv[PVT_index] = Yv_p;
            Tv[PVT_index] = Tv_p;
            PVTsegmentDuration[PVT_index] = PVTsegmentDuration_p;
        }

    }

    public void calculateCoefficients(int N){
        moveCounter = 0;

        if (N<1){
            throw new RuntimeException("n is less than 1");
        }


        xInitPosition = Xp[N-1];
        xFinalPosition = Xp[N];
        xInitVelocity = Xv[N-1];
        xFinalVelocity = Xv[N];

        yInitPosition = Yp[N-1];
        yFinalPosition = Yp[N];
        yInitVelocity = Yv[N-1];
        yFinalVelocity = Yv[N];

        thetaInitPosition =  Tp[N-1];
        thetaFinalPosition = Tp[N];
        thetaInitVelocity = Tv[N-1];
        thetaFinalVelocity = Tv[N];

        segmentDuration = PVTsegmentDuration[N];

        xZero = xInitPosition ;
        xFirst =xInitVelocity ;
        xSecond = (3*(xFinalPosition - xInitPosition)/Math.pow(segmentDuration,2) - ((2*xInitVelocity) + xFinalVelocity)/segmentDuration);
        xThird = ((xFinalVelocity+xInitVelocity)/Math.pow(segmentDuration,2)) + ((2*(xInitPosition-xFinalPosition))/Math.pow(segmentDuration,3));

        // xThird = (2*(xFinalPosition - xInitPosition  - xInitVelocity*segmentDuration)-(xFinalVelocity-xInitVelocity)*Math.pow(segmentDuration,2))/Math.pow(segmentDuration,3);
        // xSecond = (xFinalVelocity - xInitVelocity - (3*xThird*Math.pow(segmentDuration,2)))/(2*segmentDuration);

        yZero = yInitPosition;
        yFirst = yInitVelocity;
        ySecond = (3*(yFinalPosition - yInitPosition)/Math.pow(segmentDuration,2) - ((2*yInitVelocity) + yFinalVelocity)/segmentDuration);
        yThird = ((yFinalVelocity+yInitVelocity)/Math.pow(segmentDuration,2)) + ((2*(yInitPosition-yFinalPosition))/Math.pow(segmentDuration,3));

        thetaZero = thetaInitPosition;
        thetaFirst = thetaInitVelocity;
        thetaSecond = (3*(thetaFinalPosition - thetaInitPosition)/Math.pow(segmentDuration,2) - ((2*thetaInitVelocity) + thetaFinalVelocity)/segmentDuration);
        thetaThird = ((thetaFinalVelocity+thetaInitVelocity)/Math.pow(segmentDuration,2)) + ((2*(thetaInitPosition-thetaFinalPosition))/Math.pow(segmentDuration,3));


    }

    public void movePath(){
        if (PVT_index <= 0){
            throw new RuntimeException("table is empty :(");
        }
        currentIndex = 1;
        curveState = CurveState.calculatingCoefficients;


    }

    public void release(){
        if (gripperMode== "external"){
            gripServo.setPosition(0.7);
        } else {
            clawServo.setPosition(0.5);
            gripServo.setPosition(0.9);
        }

    }

    public void grip(){
        if (gripperMode== "external"){
            gripServo.setPosition(0.9);
        } else {
            gripServo.setPosition(0.7);
        }

    }



    public boolean isCloseToTarget(double targetBotx, double targetBoty, double botx, double boty, double tolerance) {
        // Calculate the absolute distance between the current position and the target position
        double xDistance = Math.abs(botx - targetBotx);  // Difference in botx
        double yDistance = Math.abs(boty - targetBoty);  // Difference in boty


        // Check if both x and y distances are within the tolerance range
        return (xDistance <= tolerance && yDistance <= tolerance);
    }



}






