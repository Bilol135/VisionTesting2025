package frc.robot.subsystems.shooter;

import static frc.robot.subsystems.drive.DriveConstants.backLeftDriveCanId;
import static frc.robot.subsystems.drive.DriveConstants.backLeftTurnCanId;
import static frc.robot.subsystems.drive.DriveConstants.backLeftZeroRotation;
import static frc.robot.subsystems.drive.DriveConstants.backRightDriveCanId;
import static frc.robot.subsystems.drive.DriveConstants.backRightTurnCanId;
import static frc.robot.subsystems.drive.DriveConstants.backRightZeroRotation;
import static frc.robot.subsystems.drive.DriveConstants.driveKs;
import static frc.robot.subsystems.drive.DriveConstants.driveKv;
import static frc.robot.subsystems.drive.DriveConstants.frontLeftDriveCanId;
import static frc.robot.subsystems.drive.DriveConstants.frontLeftTurnCanId;
import static frc.robot.subsystems.drive.DriveConstants.frontLeftZeroRotation;
import static frc.robot.subsystems.drive.DriveConstants.frontRightDriveCanId;
import static frc.robot.subsystems.drive.DriveConstants.frontRightTurnCanId;
import static frc.robot.subsystems.drive.DriveConstants.frontRightZeroRotation;
import static frc.robot.subsystems.drive.DriveConstants.odometryFrequency;

import java.util.Queue;
import java.util.function.DoubleSupplier;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.Queue;
import java.util.function.DoubleSupplier;
import frc.robot.subsystems.drive.ModuleIO.ModuleIOInputs;
import frc.robot.subsystems.drive.SparkOdometryThread;

public class ShooterIOSpark implements ShooterIO {
    private final Rotation2d zeroRotation;

  // Hardware objects
  private final SparkBase pivotSpark;
  private final RelativeEncoder pivotEncoder;

  // Closed loop controllers
  private final SparkClosedLoopController pivotController;

//   // Queue inputs from odometry thread
//   private final Queue<Double> timestampQueue;
//   private final Queue<Double> drivePositionQueue;
//   private final Queue<Double> turnPositionQueue;

  // Connection debouncers
  private final Debouncer pivotConnectedDebounce = new Debouncer(0.5);

  public ShooterIOSpark() {

    pivotSpark = new SparkMax(ShooterConstants.kPivotCANID, MotorType.kBrushless);
    
    pivotEncoder = pivotSpark.getEncoder();
    pivotController = pivotSpark.getClosedLoopController();

    // Configure drive motor
    var pivotConfig = new SparkFlexConfig();
    pivotConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(20)
        .voltageCompensation(12.0);
    pivotConfig
        .encoder
        .positionConversionFactor(1)
        .velocityConversionFactor(1);
    pivotConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pidf(
            ShooterConstants.kPivotP, 0.0,
            ShooterConstants.kPivotD, 0.0);


}

@Override
public void setPivotSpeed(double speed) {
    pivotSpark.set(speed);
}

@Override
public void setPivotPosition(double setpoint) {
   pivotEncoder.setPosition(setpoint);
}

@Override
public double getPivotPosition() {
    return pivotEncoder.getPosition();
}


@Override
public void reset() {
    pivotEncoder.setPosition(0);
}



 
}   
