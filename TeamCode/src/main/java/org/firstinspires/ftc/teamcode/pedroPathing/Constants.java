package org.firstinspires.ftc.teamcode.pedroPathing;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.FilteredPIDFCoefficients;



import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.PedroPIDTuning;

public class Constants {

//    private static double kP = PedroPIDTuning.kP;   //TranslationalPID
//    private static double kI = PedroPIDTuning.kI;   //TranslationalPID
//    private static double kD = PedroPIDTuning.kD;   //TranslationalPID
//    private static double kF = 0.0305;   //TranslationalPID





    private static FollowerConstants followerConstants = new FollowerConstants()
            .mass(11.8)
            .forwardZeroPowerAcceleration(-24.756019393735684)
            .lateralZeroPowerAcceleration(-65.64511813017555)
            .translationalPIDFCoefficients(new PIDFCoefficients(
                    0.1,   //0.1 old version (first sorter)
                    0.0,
                    0.011,  //0.015
                    0.035     //0.036
            ))
            .useSecondaryTranslationalPIDF(false)
            .headingPIDFCoefficients(new PIDFCoefficients(
                    0.8,
                    0,
                    0.075,
                    0.024
                    ))
            .useSecondaryHeadingPIDF(false)
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(
                    0.017,
                    0.0,
                    0.0007,
                    0.6,
                    0.0
            ))
            .useSecondaryDrivePIDF(false)
            .centripetalScaling(0.0003);



    private static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("FR")
            .rightRearMotorName("BR")
            .leftRearMotorName("BL")
            .leftFrontMotorName("FL")
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .xVelocity( 86.49005367249015)     //83.287727416031
            .yVelocity(70.86871722364052);  //65.79496705062745 65.46490286278912



    private static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(54.538)
            .strafePodX(-176.462)
            .distanceUnit(DistanceUnit.MM)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);
            //-19959.794547170917

    private static PathConstraints pathConstraints = new PathConstraints(
            0.99,
            100,
            1,
            1);

    public static void updateFollowerPID() {

//        followerConstants.setCoefficientsDrivePIDF(new FilteredPIDFCoefficients(
//                PedroPIDTuning.kP,
//                PedroPIDTuning.kI,
//                PedroPIDTuning.kD,
//                0.6,
//                PedroPIDTuning.kF
//        ));
        /*
        followerConstants.setCoefficientsHeadingPIDF(new PIDFCoefficients(
                PedroPIDTuning.kP,
                PedroPIDTuning.kI,
                PedroPIDTuning.kD,
                PedroPIDTuning.kF

        ));*/
        //followerConstants.setCentripetalScaling(PedroPIDTuning.centripicalScaling);
    }

    public void setMaxPower(double maxPower){
        driveConstants.setMaxPower(maxPower);
    }

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }

}
