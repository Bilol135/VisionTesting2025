package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {

    private final ShooterIO shooter;

    public Shooter(ShooterIO shooter) {
        this.shooter = shooter;
    }

    //shooter pivot up
    public Command shooterPivotUp() {
        return run(
            () -> shooter.setPivotSpeed(1));
    }
    //shooter pivot down
    public Command shooterPivotDown() {
        return run(
            () -> shooter.setPivotSpeed(-1));
    }
    //stops shooter pivot
    public Command stopShooterPivot() {
        return run(
            () -> shooter.setPivotSpeed(0));
    }

    public void resetEncoder() {
        shooter.reset();
    }



    @Override
    public void periodic() {
        //SmartDashboard.putNumber("Intake Position", Math.round(intake.getPivotPosition() * Math.pow(10, 2)) / Math.pow(10, 2));
    }
}
