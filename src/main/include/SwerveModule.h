
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/MathUtil.h>
#include <ctre/phoenix/sensors/WPI_CANCoder.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include "Constants.h"
#include <units/angle.h>
#include <rev/CANSparkMax.h>
#include <units/math.h>
#include <ctre/phoenix/motorcontrol/SupplyCurrentLimitConfiguration.h>
#include <ctre/phoenix/motorcontrol/can/TalonFX.h>
#include <ctre/phoenix/sensors/AbsoluteSensorRange.h>
#include <ctre/phoenix/sensors/SensorInitializationStrategy.h>
#include <ctre/phoenix/sensors/SensorTimeBase.h>
// #include <frc/math/controller/SimpleMotorFeedforward.h>


// #include <ctre/phoenix/motorcontrol/can/WPI_TalonFX.h>
// #include "HardwareConfig.h"
// #include "<CANEncoder.h>"
// #include <AbsoluteEncoder.h>
// #include "rev/SparkMaxPIDController.h"

// #include <ctre/ANSparkMax.h>
// #include "rev/Cphoenix/sensors/CANCoder.h>
// #include <rev/CANSparkMaxLowLevel.h"
// #include "rev/RelativeEncoder.h"
// #include <frc/math/geometry/Rotation2d.h>
// #include <frc/math/kinematics/SwerveModuleState.h>
// #include "frc/lib/config/SwerveModuleConstants.h"
// #include "frc/lib/math/OnboardModuleState.h"
// #include "frc/lib/util/CANCoderUtil.h"
// #include "frc/lib/util/CANSparkMaxUtil.h"
// #include "frc/robot/Constants.h"

class SwerveModule {
    public:
        SwerveModule(const double Module[]);
        void SetDesiredState(frc::SwerveModuleState& DesiredState, bool IsOpenLoop);
        void SetDesiredAngle(frc::Rotation2d Angle);
        frc::Rotation2d GetCANCoder();
        frc::SwerveModuleState Optimize(frc::SwerveModuleState DesiredState, frc::Rotation2d CurrentAngle);
        frc::SwerveModuleState GetState();
        void SwapOrientation();   
        frc::SwerveModulePosition GetPosition();
        void ResetToAbsolute();

        units::meter_t NEOToMeters(double Counts);    
        units::degree_t NEOToDegrees(double Counts);
        double DegreesToNEO(units::degree_t Degrees);
        // double NEOToRPM(double VelocityCounts);
        // double RPMToNEO(double RPM);
        double getTurnCounts();
        units::degree_t getLastAngle();
        units::meters_per_second_t RPMToMPS(double Velocitycounts);
        // double MPSToNEO(units::meters_per_second_t Velocity);
        
    private:
        
        rev::CANSparkMax m_DriveMotor;
        rev::CANSparkMax m_AngleMotor;
        
        ctre::phoenix::sensors::WPI_CANCoder m_AngleCANcoder;
        
        units::degree_t m_AngleOffset;

        rev::SparkMaxPIDController m_DrivePID;
        rev::SparkMaxPIDController m_AnglePID;

        rev::SparkMaxRelativeEncoder m_DriveRelEncoder;
        rev::SparkMaxRelativeEncoder m_AngleRelEncoder;

        frc::SimpleMotorFeedforward<units::feet> m_Feedforward;
        
        units::degree_t m_LastAngle;
        
        ctre::phoenix::sensors::CANCoderConfiguration m_SwerveCanCoderConfig;
        
        // units::degree_t m_LastAngle;
        // ctre::phoenix::motorcontrol::can::WPI_TalonFX m_DriveMotor;
        // ctre::phoenix::motorcontrol::can::WPI_TalonFX m_AngleMotor;
        // ctre::phoenix::sensors::WPI_CANCoder m_AngleEncoder;
        
        // HardwareConfig m_Settings;

        // rev::CANSparkMax SparkMax{1, rev::CANSparkMax::MotorType::kBrushless};
        // SparkMax.RestoreFactoryDefaults();
        // rev::CANEncoder CanCoder = SparkMax.GetEncoder(rev::CANEncoder::EncoderType::kQuadrature);
        // CanCoder.SetInverted(true);
        // SparkMax.SetFeedBackDevice(CanCoder);
        
        // rev::CANPIDController pidController = SparkMax.GetPIDController();
        // pidController.SetFeedbackDevice(CanCoder);

        // pidController.SetP(kP);
        // pidController.SetI(kI);
        // pidController.setD(kD);
        // pidController.SetIZone(kIz);
        // pidController.SetFF(kFF);
        // pidController.SetOutputRange(kMin, kMax);

};