package frc.robot;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Angle;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import frc.lib.math.Conversions;
import frc.lib.util.SwerveModuleConstants;

public class SwerveModule {
    public int moduleNumber;
    private Rotation2d lastAngle;
    private Rotation2d angleOffset;

    // private TalonFX mAngleMotor;
    // private TalonFX mDriveMotor;

    private CANSparkMax angleMotor;
    private CANSparkMax driveMotor;

    private CANcoder angleEncoder;
    private RelativeEncoder driveEncoder;
    private RelativeEncoder integratedAngleEncoder;

    private final SparkPIDController driveController;
    private final SparkPIDController angleController;

    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA);

    //private final SimpleMotorFeedforward driveFeedForward = new SimpleMotorFeedforward(Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA);

    /* drive motor control requests */
    private final DutyCycleOut driveDutyCycle = new DutyCycleOut(0);
    private final VelocityVoltage driveVelocity = new VelocityVoltage(0);

    /* angle motor control requests */
    private final PositionVoltage anglePosition = new PositionVoltage(0);

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants){
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;
        
        /* Angle Encoder Config */
        angleEncoder = new CANcoder(moduleConstants.cancoderID);
        angleEncoder.getConfigurator().apply(Robot.ctreConfigs.swerveCANcoderConfig);

        /* Angle Motor Config */
        // mAngleMotor = new TalonFX(moduleConstants.angleMotorID);
        // mAngleMotor.getConfigurator().apply(Robot.ctreConfigs.swerveAngleFXConfig);
        // resetToAbsolute();
        angleMotor = new CANSparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
        integratedAngleEncoder = angleMotor.getEncoder();
        angleController = angleMotor.getPIDController();
        configAngleMotor();
        resetToAbsolute();

        /* Drive Motor Config */
        // mDriveMotor = new TalonFX(moduleConstants.driveMotorID);
        // mDriveMotor.getConfigurator().apply(Robot.ctreConfigs.swerveDriveFXConfig);
        // mDriveMotor.getConfigurator().setPosition(0.0);
        driveMotor = new CANSparkMax(moduleConstants.driveMotorID, MotorType.kBrushless);
        driveEncoder = driveMotor.getEncoder();
        driveController = driveMotor.getPIDController();
        resetToAbsolute();
    }

    private void configAngleMotor() {
        angleMotor.restoreFactoryDefaults();
        angleMotor.setSmartCurrentLimit(Constants.Swerve.angleContinuousCurrentLimit);
        integratedAngleEncoder.setPositionConversionFactor(Constants.Swerve.angleConversionFactor);
        angleController.setPositionPIDWrappingEnabled(true);
        angleController.setPositionPIDWrappingMinInput(-180.0);
        angleController.setPositionPIDWrappingMaxInput(180.0);
        angleController.setP(Constants.Swerve.angleKP);
        angleController.setI(Constants.Swerve.angleKI);
        angleController.setD(Constants.Swerve.angleKD);
        angleMotor.enableVoltageCompensation(Constants.Swerve.voltageComp);
        angleMotor.burnFlash();
        resetToAbsolute();
      }

      private void configDriveMotor() {
        driveMotor.restoreFactoryDefaults();
        driveMotor.setSmartCurrentLimit(Constants.Swerve.driveContinuousCurrentLimit);
        driveEncoder.setPositionConversionFactor(0.060509807);
        driveEncoder.setVelocityConversionFactor(Constants.Swerve.driveConversionVelocityFactor);
        driveController.setP(Constants.Swerve.angleKP);
        driveController.setI(Constants.Swerve.angleKI);
        driveController.setD(Constants.Swerve.angleKD);
        driveMotor.enableVoltageCompensation(Constants.Swerve.voltageComp);
        driveMotor.burnFlash();
        driveEncoder.setPosition(0.0);
      }

      private void setAngle(SwerveModuleState desiredState) {
        // Prevent rotating module if speed is less then 1%. Prevents jittering.
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.maxSpeed * 0.01))
            ? lastAngle
            : desiredState.angle;
    
        angleController.setReference(angle.getDegrees(), ControlType.kPosition);
        lastAngle = angle;
      }
    

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
        desiredState = SwerveModuleState.optimize(desiredState, getState().angle); 
        // mAngleMotor.setControl(anglePosition.withPosition(desiredState.angle.getRotations()));
        // setSpeed(desiredState, isOpenLoop);

        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop){
        // if(isOpenLoop){
        //     driveDutyCycle.Output = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
        //     mDriveMotor.setControl(driveDutyCycle);
        // }
        // else {
        //     driveVelocity.Velocity = Conversions.MPSToRPS(desiredState.speedMetersPerSecond, Constants.Swerve.wheelCircumference);
        //     driveVelocity.FeedForward = driveFeedForward.calculate(desiredState.speedMetersPerSecond);
        //     mDriveMotor.setControl(driveVelocity);
        // }

        if (isOpenLoop) {
            double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
            driveMotor.set(percentOutput);
          } else {
            driveController.setReference(
                desiredState.speedMetersPerSecond,
                ControlType.kVelocity,
                0,
                feedforward.calculate(desiredState.speedMetersPerSecond));
          }
    }

    public Rotation2d getCANcoder(){
        return Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValue());
        //return Rotation2d.fromDegrees(moduleNumber)
    }

    public void resetToAbsolute(){
        double absolutePosition = getCANcoder().getRotations() - angleOffset.getRotations();
        //mAngleMotor.setPosition(absolutePosition);
        integratedAngleEncoder.setPosition(absolutePosition);

    }

    private Rotation2d getAngle() {
        return Rotation2d.fromDegrees(integratedAngleEncoder.getPosition());
      }

    public SwerveModuleState getState(){
        // return new SwerveModuleState(
        //     Conversions.RPSToMPS(mDriveMotor.getVelocity().getValue(), Constants.Swerve.wheelCircumference), 
        //     Rotation2d.fromRotations(mAngleMotor.getPosition().getValue())
        // );
        double velocity = driveEncoder.getVelocity();
        return new SwerveModuleState(velocity, getAngle());

    }

    public SwerveModulePosition getPosition(){
        // return new SwerveModulePosition(
        //     Conversions.rotationsToMeters(mDriveMotor.getPosition().getValue(), Constants.Swerve.wheelCircumference), 
        //     Rotation2d.fromRotations(mAngleMotor.getPosition().getValue())
        // );

        double position = driveEncoder.getPosition();
        return new SwerveModulePosition(position, getAngle());
    }
}