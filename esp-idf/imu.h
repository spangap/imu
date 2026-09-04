/**
 * imu — motion sensing, and nothing else.
 *
 * A QMI8658 accelerometer (SPI or I2C, whichever the board wired — CONFIG_IMU_*)
 * held in low-power mode with Wake-on-Motion armed. The task turns the part's
 * latched motion events into a debounced answer to one question — is this
 * device moving — and publishes it to ephemeral imu.*. See imu.cpp for the
 * register sequences.
 *
 * Published keys (the contract other straddles use; nobody links against this):
 *
 *   imu.present  0/1     the part answered at boot
 *   imu.model    string  "QMI8658", or empty
 *   imu.moving   0/1     debounced: 1 on a motion event, 0 after s.imu.still_s
 *                        without one
 *   imu.motion   int     per-boot event counter — for a consumer that may have
 *                        been asleep through the edge, since a counter cannot
 *                        be missed the way a level can
 *   imu.still_s  int     seconds since the last motion event
 *   imu.state    string  finished text for a pane: "moving", "still 4m",
 *                        "asleep", "not present"
 *
 * Config: s.imu.enable (0/1), s.imu.threshold_mg (motion threshold, lower =
 * more sensitive), s.imu.odr (accelerometer rate in Hz: 3/11/21/128),
 * s.imu.still_s (how long without an event counts as still).
 */
#pragma once
#include <stdint.h>
#include "service.h"

/** True iff the part answered at boot. */
bool     imuPresent(void);

/** Debounced motion state — the same fact as imu.moving, for a caller inside
 *  this image that would rather not go through storage. */
bool     imuMoving(void);

/** Per-boot motion-event counter (imu.motion). */
uint32_t imuMotionCount(void);

/** Seconds since the last motion event (imu.still_s); 0 while moving. */
uint32_t imuStillSeconds(void);

/** The IMU as a boot-registered Service: onInit probes the part, arms
 *  Wake-on-Motion and spawns the motion task. Declared in imu straddle.yaml
 *  `services:`. */
class ImuService : public Service {
public:
    void onInit() override;
};
