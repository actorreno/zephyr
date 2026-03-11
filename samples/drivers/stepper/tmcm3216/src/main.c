/*
 * SPDX-FileCopyrightText: Copyright (c) 2025 Alexis Czezar C Torreno
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/drivers/stepper/stepper.h>
#include <zephyr/drivers/stepper/stepper_ctrl.h>
#include <zephyr/drivers/stepper/stepper_tmcm3216.h>
#include <zephyr/kernel.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(stepper_tmcm3216, CONFIG_STEPPER_LOG_LEVEL);

static const struct device *stepper_drv = DEVICE_DT_GET(DT_NODELABEL(stepper_driver_0));
static const struct device *stepper_ctl = DEVICE_DT_GET(DT_NODELABEL(stepper_ctrl_0));

static K_SEM_DEFINE(steps_completed_sem, 0, 1);

static int32_t ping_pong_target = CONFIG_STEPS_PER_REV * CONFIG_PING_PONG_N_REV *
				  DT_PROP(DT_NODELABEL(stepper_driver_0), micro_step_res);

static void stepper_callback(const struct device *dev, const enum stepper_ctrl_event event,
			     void *user_data)
{
	ARG_UNUSED(user_data);
	switch (event) {
	case STEPPER_CTRL_EVENT_STEPS_COMPLETED:
		k_sem_give(&steps_completed_sem);
		break;
	default:
		break;
	}
}

static void print_tmcm3216_status(const struct device *dev)
{
	struct tmcm3216_status status;

	if (tmcm3216_get_status(dev, &status) == 0) {
		LOG_INF("Status: pos=%d vel=%d moving=%s pos_reached=%s left_end=%s right_end=%s",
			status.actual_position, status.actual_velocity,
			status.is_moving ? "yes" : "no",
			status.position_reached ? "yes" : "no",
			status.left_endstop ? "yes" : "no",
			status.right_endstop ? "yes" : "no");
	}
}

int main(void)
{
	LOG_INF("Starting TMCM-3216 stepper sample");

	if (!device_is_ready(stepper_ctl)) {
		LOG_ERR("Stepper ctrl device %s not ready", stepper_ctl->name);
		return -ENODEV;
	}

	if (!device_is_ready(stepper_drv)) {
		LOG_ERR("Stepper driver device %s not ready", stepper_drv->name);
		return -ENODEV;
	}

	stepper_ctrl_set_event_cb(stepper_ctl, stepper_callback, NULL);
	stepper_enable(stepper_drv);

	/* TMCM-3216 specific: configure velocity and acceleration */
	tmcm3216_set_max_velocity(stepper_ctl, CONFIG_MAX_VELOCITY);
	LOG_INF("Max velocity set to %d usteps/s", CONFIG_MAX_VELOCITY);

	tmcm3216_set_max_acceleration(stepper_ctl, CONFIG_MAX_ACCELERATION);
	LOG_INF("Max acceleration set to %d usteps/s^2", CONFIG_MAX_ACCELERATION);

	/* TMCM-3216 specific: verify velocity readback */
	uint32_t max_velocity;

	tmcm3216_get_max_velocity(stepper_ctl, &max_velocity);
	LOG_DBG("Max velocity readback: %u usteps/s", max_velocity);

	stepper_ctrl_set_reference_position(stepper_ctl, 0);
	stepper_ctrl_move_by(stepper_ctl, ping_pong_target);

	for (;;) {
		if (k_sem_take(&steps_completed_sem, K_FOREVER) == 0) {
			/* TMCM-3216 specific: read extended status */
			print_tmcm3216_status(stepper_ctl);

			/* TMCM-3216 specific: read actual velocity */
			int32_t actual_vel;

			if (tmcm3216_get_actual_velocity(stepper_ctl, &actual_vel) == 0) {
				LOG_DBG("Actual velocity at stop: %d usteps/s", actual_vel);
			}

			ping_pong_target *= -1;
			stepper_ctrl_move_by(stepper_ctl, ping_pong_target);
			LOG_INF("Moving by %d steps", ping_pong_target);
		}
	}

	return 0;
}
