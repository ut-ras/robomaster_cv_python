#include "ghost_v5/motor/ghost_motor.hpp"

#include "pros/apix.h"
#include "pros/error.h"

namespace ghost_v5
{
    GhostMotor::GhostMotor(
        int motor_port,
        bool reversed,
        GhostMotorConfig &config)
        : pros::Motor(
              motor_port,
              pros::E_MOTOR_GEARSET_INVALID,
              reversed,
              config.motor__encoder_units),
          velocity_filter_(
              config.filter__cutoff_frequency,
              config.filter__damping_ratio,
              config.filter__timestep),
          motor_model_(
              config.motor__nominal_free_speed,
              config.motor__stall_torque,
              config.motor__free_current,
              config.motor__stall_current,
              config.motor__max_voltage,
              config.motor__gear_ratio),
          motor_is_3600_cart_{(config_.motor__gear_ratio == 36)},
          device_connected_{false},
          ctl_mode_{control_mode_e::VOLTAGE_CONTROL},
          des_voltage_norm_{0.0},
          des_vel_rpm_{0.0},
          des_pos_encoder_{0},
          cmd_voltage_mv_{0.0},
          is_active_{false}
    {
        config_ = config;
        trq_lim_norm_ = config_.motor__torque_limit_norm;
        ctl_rpm_deadband_ = config_.ctl__rpm_deadband;

        // Doing this in the constructor causes data abort exception
        set_gearing(RPM_TO_GEARING[config_.motor__gear_ratio]);

        if(config.motor__gear_ratio == 36.0){
            motor_is_3600_cart_ = true;
        }

        // Set Brake Mode
        set_brake_mode(config.motor__brake_mode);
    }

    void GhostMotor::updateMotor()
    {
        // Get Motor Velocity, checking for errors
        double raw_vel = get_actual_velocity();
        device_connected_ = (raw_vel != PROS_ERR_F);
        if(raw_vel == PROS_ERR_F){
            raw_vel = 0.0;
        }
        
        // Adjust reported velocity for direct drive motor gearing
        auto true_vel = (motor_is_3600_cart_) ?  6*raw_vel : raw_vel;
        
        // Update Low Pass Filter with velocity measurement
        curr_vel_rpm_ = velocity_filter_.updateFilter(true_vel);

        // Update DC Motor Model
        motor_model_.setMotorSpeedRPM(curr_vel_rpm_);

        // Calculate control inputs
        float voltage_feedforward = des_voltage_norm_ * config_.motor__max_voltage * 1000 * config_.ctl__ff_voltage_gain;
        float velocity_feedforward = motor_model_.getVoltageFromVelocityMillivolts(des_vel_rpm_) * config_.ctl__ff_vel_gain;
        float velocity_feedback = (des_vel_rpm_ - curr_vel_rpm_) * config_.ctl__vel_gain;
        float position_feedback = (des_pos_encoder_ - get_position()) * config_.ctl__pos_gain;

        // Set voltage command based on control mode
        switch (ctl_mode_)
        {
        case control_mode_e::POSITION_CONTROL:
            cmd_voltage_mv_ = voltage_feedforward + velocity_feedforward + velocity_feedback + position_feedback;
            break;

        case control_mode_e::VELOCITY_CONTROL:
            cmd_voltage_mv_ = voltage_feedforward + velocity_feedforward + velocity_feedback;
            // Apply velocity deadband
            if(fabs(des_vel_rpm_) < ctl_rpm_deadband_ && fabs(ctl_rpm_deadband_) > 1e-3){
                cmd_voltage_mv_ = 0.0;
            }
            break;

        case control_mode_e::VOLTAGE_CONTROL:
            cmd_voltage_mv_ = voltage_feedforward;
            break;
        }

        if(is_active_){
            move_voltage(cmd_voltage_mv_);
        }
        else{
            set_current_limit(0);
            move_voltage(0);
        }
    }

    void GhostMotor::move_voltage_trq_lim(float voltage_mv)
    {
        // Normalize voltage command from millivolts
        double voltage_normalized = voltage_mv / config_.motor__max_voltage / 1000.0;

        // Normalize velocity by nominal free speed
        double curr_vel_normalized = curr_vel_rpm_ / (config_.motor__nominal_free_speed * config_.motor__gear_ratio);

        // Motor torque is proportional to armature current, which is approximated by difference between back EMF
        // and driving voltage assuming constant resistance.
        // Limiting voltage difference prevents voltage spikes and prolongs motor life when changing speed rapidly.
        double cmd;
        if (fabs(curr_vel_normalized - voltage_normalized) > trq_lim_norm_)
        {
            double sign = (curr_vel_normalized - voltage_normalized > 0) ? 1.0 : -1.0;
            cmd = curr_vel_normalized - sign * fabs(trq_lim_norm_);
        }
        else
        {
            cmd = voltage_normalized;
        }

        // Limit cmd to voltage bounds
        cmd = std::min(cmd, config_.motor__max_voltage * 1000.0);
        cmd = std::max(cmd, -config_.motor__max_voltage * 1000.0);

        // Set motor
        move_voltage(cmd * config_.motor__max_voltage * 1000);
    }

    void GhostMotor::setMotorCommand(float voltage, float velocity, float position)
    {
        ctl_mode_ = control_mode_e::POSITION_CONTROL;

        des_pos_encoder_ = position;
        des_vel_rpm_ = velocity;
        des_voltage_norm_ = voltage;
    }

    void GhostMotor::setMotorCommand(float voltage, float velocity)
    {
        ctl_mode_ = control_mode_e::VELOCITY_CONTROL;

        des_vel_rpm_ = velocity;
        des_voltage_norm_ = voltage;
    }

    void GhostMotor::setMotorCommand(float voltage)
    {
        ctl_mode_ = control_mode_e::VOLTAGE_CONTROL;
        des_voltage_norm_ = voltage;
    }

} // namespace ghost_motor