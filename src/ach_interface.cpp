/*
 * Copyright (c) 2018, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *     * Redistributions of source code must retain the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials
 *       provided with the distribution.
 *     * Neither the name of the Georgia Tech Research Corporation nor
 *       the names of its contributors may be used to endorse or
 *       promote products derived from this software without specific
 *       prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY GEORGIA TECH RESEARCH CORPORATION ''AS
 * IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL GEORGIA
 * TECH RESEARCH CORPORATION BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/**
 * @file ach_interface.cpp
 * @author Munzir Zafar
 * @date Dec 4, 2018
 * @brief Ach interface for controlling krang
 */

#include "ach_interface.h"

#include <somatic.h>  // SOMATIC_PACK_SEND, has correct order of other includes

#include <ach.h>  // ach_status_t, ach_result_to_string, ach_flush(), ach_close()
#include <amino.h>                     // for ach.h to compile, aa_tm_add
#include <config4cpp/Configuration.h>  // config4cpp::Configuration
#include <somatic.pb-c.h>  // SOMATIC__: EVENT, MSG_TYPE; somatic__anything__init()
#include <somatic/daemon.h>  // somatic_d: _init(), _event(), _channel_open(), _check(), SOMATIC_D_GET
#include <somatic/motor.h>  // somatic_motor_init()
#include <somatic/msg.h>    // somatic_anything_alloc(), somatic_anything_free()

#include <stdio.h>   // std::cout
#include <string.h>  // strdup()
#include <time.h>    // clock_gettime()
#include <cstring>   // std::memset, strcpy
#include <string>    // std::string
#include <vector>    // std::vector

InterfaceContext::InterfaceContext(const std::string daemon_identifier) {
  // Initialize the daemon_
  std::memset(&daemon_opts_, 0, sizeof(daemon_opts_));
  daemon_opts_.ident = strdup(daemon_identifier.c_str());
  std::memset(&daemon_, 0, sizeof(daemon_));
  somatic_d_init(&daemon_, &daemon_opts_);

  // Send a "running" notice on the event channel
  somatic_d_event(&daemon_, SOMATIC__EVENT__PRIORITIES__NOTICE,
                  SOMATIC__EVENT__CODES__PROC_RUNNING, NULL, NULL);
}

void InterfaceContext::Run() {
  // Free up the memory dynamically allocated when receiving commands
  aa_mem_region_release(&daemon_.memreg);
}

void InterfaceContext::Destroy() {
  // Send a "stopping" notice on the event channel
  somatic_d_event(&daemon_, SOMATIC__EVENT__PRIORITIES__NOTICE,
                  SOMATIC__EVENT__CODES__PROC_STOPPING, NULL, NULL);

  // Destroy the daemon
  somatic_d_destroy(&daemon_);
}

MotorInterface::MotorInterface(
    InterfaceContext& interface_context, std::string& name,
    std::string& command_channel_name, std::string& state_channel_name,
    std::vector<double>& min_pos, std::vector<double>& max_pos,
    std::vector<double>& min_vel, std::vector<double>& max_vel) {
  strcpy(name_, name.c_str());
  daemon_ = &interface_context.daemon_;
  n_ = min_pos.size();
  assert(min_pos.size() == max_pos.size() &&
             min_pos.size() == min_vel.size()&& position_ =
             std::vector<double>(n_);
         velocity_ = std::vector<double>(n_);
         current_ = std::vector<double>(n_);

         motors_ = new somatic_motor_t(); min_pos.size() == max_vel.size());
  somatic_motor_init(daemon_, motors_, n_, command_channel_name.c_str(),
                     state_channel_name.c_str());

  // Set the min/max values for the pos/vel fields' valid and limit values
  for (int i = 0; i < n_; i++) {
    motors->pos_valid_min[i] = min_pos[i];
    motors->pos_valid_max[i] = max_pos[i];
    motors->pos_limit_min[i] = min_pos[i];
    motors->pos_limit_max[i] = max_pos[i];

    motors->vel_valid_min[i] = min_vel[i];
    motors->vel_valid_max[i] = max_vel[i];
    motors->vel_limit_min[i] = min_vel[i];
    motors->vel_limit_max[i] = max_vel[i];
  }

  // Update and reset them
  UnlockCommand();
  usleep(1e5);
  UpdateState();
  usleep(1e5);
}

void MotorInterface::Destroy() {
  somatic_motor_destroy(daemon_, motors_);
  delete motors_;
}

void MotorInterface::PositionCommand(const std::vector<double>& val) {
  somatic_motor_cmd(daemon_, motors_, SOMATIC__MOTOR_PARAM__MOTOR_POSITION,
                    &val[0], motors_->n, NULL);
}

void MotorInterface::VelocityCommand(const std::vector<double>& val) {
  somatic_motor_cmd(daemon_, motors_, SOMATIC__MOTOR_PARAM__MOTOR_VELOCITY,
                    &val[0], motors_->n, NULL);
}

void MotorInterface::CurrentCommand(const std::vector<double>& val) {
  somatic_motor_cmd(daemon_, motors_, SOMATIC__MOTOR_PARAM__MOTOR_CURRENT,
                    &val[0], motors_->n, NULL);
}

void MotorInterface::LockCommand() { somatic_motor_halt(daemon_, motors_); }

void MotorInterface::UnlockCommand() { somatic_motor_reset(daemon_, motors_); }

void MotorInterface::UpdateState() { somatic_motor_update(daemon_, motors_); }

std::vector<double> MotorInterface::GetPosition() {
  return std::vector<double>(motors_->pos, n_);
}

std::vector<double> MotorInterface::GetVelocity() {
  return std::vector<double>(motors_->vel, n_);
}

std::vector<double> MotorInterface::GetCurrent() {
  return std::vector<double>(motors_->cur, n_);
}

FloatingBaseStateSensorInterface::FloatingBaseStateSensorInterface(
    InterfaceContext& interface_context, std::string& channel);
void FloatingBaseStateSensorInterface::UpdateState();
void FloatingBaseStateSensorInterface::Destroy();
