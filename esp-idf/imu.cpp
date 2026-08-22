/**
 * imu — QMI8658 motion oracle. See imu.h for the published contract.
 *
 * The part runs its accelerometer alone, in low power, with Wake-on-Motion
 * armed: a movement past s.imu.threshold_mg latches a bit in STATUS1 and (if
 * the board wired one) toggles an interrupt line. The latch is cleared by
 * READING the register, which is what lets this work with no interrupt at all —
 * a poll can be late but cannot miss an event.
 *
 * Register facts, from the QMI8658C data sheet (rev 0.9):
 *
 *   WHO_AM_I 0x00 = 0x05           CTRL7  0x08  bit0 aEN, bit1 gEN
 *   CTRL1    0x02  bit0 SensorDisable (1 = internal 2 MHz oscillator off),
 *                  bit5 BE, bit6 ADDR_AI, bit7 SIM (0 = 4-wire SPI)
 *   CTRL2    0x03  bits 6:4 aFS, bits 3:0 aODR (0xC..0xF = the low-power rates)
 *   CTRL8    0x09  bit7 CTRL9 handshake source (1 = STATUSINT, not INT1)
 *   CTRL9    0x0A  command register; 0x08 = write WoM setting
 *   CAL1_L   0x0B  WoM threshold in mg (0x00 disables WoM)
 *   CAL1_H   0x0C  bits 7:6 interrupt select+initial level, bits 5:0 blanking
 *   STATUSINT 0x2D bit7 CmdDone, cleared on read (the CTRL9 handshake)
 *   STATUS1  0x2F  bit2 WoM  (§9.3; the register table calls bit 2 reserved —
 *                  the functional chapter is the one that is right)
 *
 * SensorDisable is bit 0, per that register table. QST's own SensorLib writes
 * bit 1 in powerDown(), which by the table is a reserved bit and leaves the
 * oscillator running.
 */
#include "imu.h"
#include "spangap.h"
#include "spi_helper.h"     /* idempotent bus init + the one GPIO ISR install */

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <cstdio>
#include <cstring>

static const char* TAG = "imu";

#define IMU_VERSION 1

/* ─────────────── QMI8658 registers ─────────────── */

#define QMI_WHOAMI        0x00
#define QMI_WHOAMI_VAL    0x05
#define QMI_CTRL1         0x02
#define QMI_CTRL1_SLEEP   0x01   /* bit0 SensorDisable */
#define QMI_CTRL2         0x03
#define QMI_CTRL7         0x08
#define QMI_CTRL7_AEN     0x01
#define QMI_CTRL8         0x09
#define QMI_CTRL8_CMD_STATUSINT 0x80
#define QMI_CTRL9         0x0A
#define QMI_CMD_WOM       0x08
#define QMI_CAL1_L        0x0B
#define QMI_CAL1_H        0x0C
#define QMI_STATUSINT     0x2D
#define QMI_STATUSINT_DONE 0x80
#define QMI_STATUS1       0x2F
#define QMI_STATUS1_WOM   0x04

/* accelerometer low-power output rates: register code -> Hz */
struct OdrCode { int hz; uint8_t code; };
static constexpr OdrCode kOdrs[] = { {128, 0x0C}, {21, 0x0D}, {11, 0x0E}, {3, 0x0F} };

/* WoM blanking: accelerometer samples ignored when the engine starts, so the
 * transient of enabling the accelerometer is not read as movement. */
static constexpr uint8_t kBlankSamples = 8;

/* ─────────────── state (single-task ownership after onInit) ─────────────── */

static spi_device_handle_t s_dev     = nullptr;
static TaskHandle_t        s_task    = nullptr;
static volatile bool       s_cfgDirty = true;
static volatile bool       s_intFired = false;

static bool     s_present   = false;
static bool     s_armed     = false;   /* WoM configured and the accelerometer on */
static bool     s_enabled   = true;
static bool     s_moving    = false;
static uint32_t s_motionCnt = 0;
static int64_t  s_lastMotionUs = 0;

static int      s_thresholdMg = 40;
static int      s_odrHz       = 11;
static int      s_stillS      = 30;
static int64_t  s_lastPublishUs = 0;

/* ─────────────── SPI ─────────────── */

static bool imuXfer(const uint8_t* tx, uint8_t* rx, size_t n) {
    spi_transaction_t t = {};
    t.length    = n * 8;
    t.tx_buffer = tx;
    t.rx_buffer = rx;
    return spi_device_transmit(s_dev, &t) == ESP_OK;
}

static uint8_t regRead(uint8_t reg) {
    uint8_t tx[2] = { (uint8_t)(reg | 0x80), 0x00 }, rx[2] = {};
    return imuXfer(tx, rx, 2) ? rx[1] : 0xFF;
}

static void regWrite(uint8_t reg, uint8_t val) {
    uint8_t tx[2] = { (uint8_t)(reg & 0x7F), val };
    imuXfer(tx, nullptr, 2);
}

/* CONFIG_IMU_SPI_HOST is the peripheral *name* (2 = SPI2/FSPI, 3 = SPI3/HSPI),
 * the spelling board headers and every other straddle's pin block use. The IDF
 * enum is offset by one (SPI1_HOST=0, SPI2_HOST=1, SPI3_HOST=2), so map by name:
 * casting the raw int puts host 3 outside the enum entirely and
 * spi_bus_initialize answers "invalid host_id". */
static_assert(CONFIG_IMU_SPI_HOST < 0 ||
              CONFIG_IMU_SPI_HOST == 2 || CONFIG_IMU_SPI_HOST == 3,
              "IMU_SPI_HOST must be 2 (SPI2) or 3 (SPI3), or -1 for not wired");
static constexpr spi_host_device_t kImuHost =
    (CONFIG_IMU_SPI_HOST == 2) ? SPI2_HOST : SPI3_HOST;

/* The host may own this bus already — spangapInit() brings it up for the SD
 * card before any service runs, and the two share it. spiHelperInitBus is the
 * platform's idempotent form of that call: the first caller's pins win, later
 * ones are a no-op, and IDF's "already initialized" error line is suppressed.
 *
 * No spiHelperBusLock here: that lock exists for esp_lcd's async DMA, which
 * releases the driver's bus lock before the transfer has drained. Between two
 * ordinary registered devices — this and the SD card — spi_master's own
 * per-bus arbitration is what serialises them. */
static bool imuBusOpen(void) {
    const spi_host_device_t host = kImuHost;
    spi_bus_config_t bus = {};
    bus.sclk_io_num     = CONFIG_IMU_SPI_SCK_PIN;
    bus.mosi_io_num     = CONFIG_IMU_SPI_MOSI_PIN;
    bus.miso_io_num     = CONFIG_IMU_SPI_MISO_PIN;
    bus.quadwp_io_num   = -1;
    bus.quadhd_io_num   = -1;
    bus.max_transfer_sz = 16;
    if (spiHelperInitBus(host, &bus) != ESP_OK) {
        warn("SPI%d would not open", CONFIG_IMU_SPI_HOST);
        return false;
    }
    /* Mode 0 or mode 3, both of which the part accepts and neither of which the
     * vendor's material commits to for a given board — so the ID read decides. */
    for (int mode : { 0, 3 }) {
        spi_device_interface_config_t dev = {};
        dev.clock_speed_hz = 1000000;
        dev.mode           = mode;
        dev.spics_io_num   = CONFIG_IMU_CS_PIN;
        dev.queue_size     = 1;
        if (spi_bus_add_device(host, &dev, &s_dev) != ESP_OK) return false;
        for (int i = 0; i < 5; i++) {          /* a cold rail needs a few ms */
            if (regRead(QMI_WHOAMI) == QMI_WHOAMI_VAL) {
                dbg("QMI8658 answers in SPI mode %d", mode);
                return true;
            }
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        spi_bus_remove_device(s_dev);
        s_dev = nullptr;
    }
    return false;
}

/* ─────────────── the part's two states ─────────────── */

/* CTRL9 commands complete asynchronously; the handshake is CmdDone in
 * STATUSINT, which the read itself clears. CTRL8 bit7 points the handshake at
 * that register instead of at INT1, which this board may not have wired. */
static bool cmd9(uint8_t command) {
    regWrite(QMI_CTRL8, (uint8_t)(regRead(QMI_CTRL8) | QMI_CTRL8_CMD_STATUSINT));
    regWrite(QMI_CTRL9, command);
    for (int i = 0; i < 20; i++) {
        if (regRead(QMI_STATUSINT) & QMI_STATUSINT_DONE) return true;
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    warn("CTRL9 command 0x%02X did not complete", command);
    return false;
}

static uint8_t odrCode(int hz) {
    for (auto& o : kOdrs) if (o.hz == hz) return o.code;
    return 0x0E;                                   /* 11 Hz */
}

/* Arm Wake-on-Motion, per the data sheet's configuration sequence: sensors off,
 * accelerometer range and rate, threshold and interrupt selection, the CTRL9
 * command, then the accelerometer on. */
static bool imuArm(void) {
    regWrite(QMI_CTRL1, (uint8_t)(regRead(QMI_CTRL1) & ~QMI_CTRL1_SLEEP));
    regWrite(QMI_CTRL7, 0x00);
    regWrite(QMI_CTRL2, odrCode(s_odrHz));         /* aFS 000 = ±2 g, the finest */

    uint8_t thr = (uint8_t)(s_thresholdMg < 1 ? 1 : s_thresholdMg > 255 ? 255 : s_thresholdMg);
    /* CAL1_H bits 7:6 select the interrupt line and its resting level; the WoM
     * event toggles that line rather than driving it to a fixed state, so the
     * resting level is a don't-care for a host that reads STATUS1. */
    uint8_t sel = (CONFIG_IMU_INT_LINE == 2) ? 0x40 : 0x00;
    regWrite(QMI_CAL1_L, thr);
    regWrite(QMI_CAL1_H, (uint8_t)(sel | kBlankSamples));
    if (!cmd9(QMI_CMD_WOM)) return false;
    regWrite(QMI_CTRL7, QMI_CTRL7_AEN);
    s_armed = true;
    info("armed: %d mg, %d Hz, still after %d s", thr, s_odrHz, s_stillS);
    return true;
}

/* Leave WoM (threshold 0 returns the interrupt pins to normal) and stop the
 * oscillator — the part's lowest-power state, and where a disabled IMU or a
 * build with nothing to tell belongs. */
static void imuSleep(void) {
    regWrite(QMI_CTRL7, 0x00);
    regWrite(QMI_CAL1_L, 0x00);
    cmd9(QMI_CMD_WOM);
    regWrite(QMI_CTRL1, (uint8_t)(regRead(QMI_CTRL1) | QMI_CTRL1_SLEEP));
    s_armed = false;
}

/* ─────────────── publish ─────────────── */

static void publishState(void) {
    char state[32];
    uint32_t stillS = s_moving ? 0 : (uint32_t)((esp_timer_get_time() - s_lastMotionUs) / 1000000);

    if (!s_present)      snprintf(state, sizeof(state), "not present");
    else if (!s_enabled) snprintf(state, sizeof(state), "asleep");
    else if (s_moving)   snprintf(state, sizeof(state), "moving");
    else if (stillS >= 3600) snprintf(state, sizeof(state), "still %luh", (unsigned long)(stillS / 3600));
    else if (stillS >= 60)   snprintf(state, sizeof(state), "still %lum", (unsigned long)(stillS / 60));
    else                     snprintf(state, sizeof(state), "still %lus", (unsigned long)stillS);

    storageBegin();
    storageSet("imu.state",   state);
    storageSet("imu.moving",  s_moving ? 1 : 0);
    storageSet("imu.still_s", (int)stillS);
    storageSet("imu.motion",  (int)s_motionCnt);
    storageEnd();
}

/* ─────────────── public API ─────────────── */

bool     imuPresent(void)      { return s_present; }
bool     imuMoving(void)       { return s_moving; }
uint32_t imuMotionCount(void)  { return s_motionCnt; }
uint32_t imuStillSeconds(void) {
    return s_moving ? 0 : (uint32_t)((esp_timer_get_time() - s_lastMotionUs) / 1000000);
}

/* ─────────────── config ─────────────── */

static void applyConfig(void) {
    bool wasEnabled = s_enabled;
    s_enabled     = storageGetInt("s.imu.enable", 1) != 0;
    int thr       = storageGetInt("s.imu.threshold_mg", 40);
    int odr       = storageGetInt("s.imu.odr", 11);
    int still     = storageGetInt("s.imu.still_s", 30);

    bool retune = (thr != s_thresholdMg) || (odr != s_odrHz);
    s_thresholdMg = thr;
    s_odrHz       = odr;
    s_stillS      = still < 1 ? 1 : still;

    if (!s_present) return;
    if (!s_enabled) {
        if (wasEnabled || s_armed) { imuSleep(); info("disabled (asleep)"); }
        s_moving = false;
        publishState();
        return;
    }
    if (!s_armed || retune) imuArm();
    publishState();
}

static void onCfgChange(const char* /*key*/, const char* /*val*/) {
    s_cfgDirty = true;
    if (s_task) xTaskNotifyGive(s_task);
}

/* ─────────────── interrupt (optional) ─────────────── */

static void IRAM_ATTR imuIsr(void*) {
    s_intFired = true;
    if (s_task) {
        BaseType_t hp = pdFALSE;
        vTaskNotifyGiveFromISR(s_task, &hp);
        if (hp) portYIELD_FROM_ISR();
    }
}

static void imuIntInit(void) {
    if (CONFIG_IMU_INT_PIN < 0) return;
    gpio_config_t io = {};
    io.pin_bit_mask = 1ULL << CONFIG_IMU_INT_PIN;
    io.mode         = GPIO_MODE_INPUT;
    io.pull_up_en   = GPIO_PULLUP_DISABLE;
    io.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io.intr_type    = GPIO_INTR_ANYEDGE;   /* the part TOGGLES the line per event */
    gpio_config(&io);
    if (spiHelperEnsureGpioIsr(0) != ESP_OK) return;   /* one install for the whole app */
    if (gpio_isr_handler_add((gpio_num_t)CONFIG_IMU_INT_PIN, imuIsr, nullptr) == ESP_OK)
        dbg("motion interrupt on GPIO%d (INT%d)", CONFIG_IMU_INT_PIN, CONFIG_IMU_INT_LINE);
}

/* ─────────────── CLI ─────────────── */

static void cliImu(const char* args) {
    if (args && strcmp(args, "help") == 0) {
        cliPrintf("  %-*s motion sensor status\n",  CLI_HELP_COL, "imu");
        cliPrintf("  %-*s enable/disable sensing\n", CLI_HELP_COL, "imu on|off");
        return;
    }
    if (args && strcmp(args, "on")  == 0) { storageSet("s.imu.enable", 1); cliPrintf("enabled\n");  return; }
    if (args && strcmp(args, "off") == 0) { storageSet("s.imu.enable", 0); cliPrintf("disabled\n"); return; }

    cliPrintf("sensor:    %s\n", s_present ? "QMI8658" : "not present");
    if (!s_present) return;
    cliPrintf("state:     %s\n", !s_enabled ? "asleep" : s_moving ? "moving" : "still");
    cliPrintf("still:     %lu s\n", (unsigned long)imuStillSeconds());
    cliPrintf("events:    %lu since boot\n", (unsigned long)s_motionCnt);
    cliPrintf("threshold: %d mg @ %d Hz\n", s_thresholdMg, s_odrHz);
    cliPrintf("still at:  %d s without an event\n", s_stillS);
    cliPrintf("interrupt: %s\n", CONFIG_IMU_INT_PIN >= 0 ? "wired (polled as backstop)" : "not wired (polled)");
}

/* ─────────────── task ─────────────── */

/* One second is the poll floor: it bounds how late a motion event can be
 * noticed, and the read is two SPI bytes. The interrupt, where a board wires
 * one, only makes the same answer arrive sooner. */
static constexpr int kPollMs     = 1000;
static constexpr int kHeartbeatS = 10;    /* republish this often even when nothing changes */

static void imuTaskMain(void*) {
    info("task up");
    itsClientInit(2);
    storageSubscribeChanges("s.imu", onCfgChange);

    for (;;) {
        if (s_cfgDirty) { s_cfgDirty = false; applyConfig(); }

        if (s_present && s_enabled && s_armed) {
            s_intFired = false;
            /* Reading STATUS1 clears the latch and releases the interrupt line,
             * so this both asks and acknowledges. */
            bool event = (regRead(QMI_STATUS1) & QMI_STATUS1_WOM) != 0;
            int64_t now = esp_timer_get_time();
            bool    changed = false;

            if (event) {
                s_motionCnt++;
                s_lastMotionUs = now;
                changed = true;
                if (!s_moving) {
                    s_moving = true;
                    info("motion detected");
                }
            } else if (s_moving && (now - s_lastMotionUs) >= (int64_t)s_stillS * 1000000) {
                s_moving = false;
                changed  = true;
                info("still for %d s", s_stillS);
            }

            /* On change, and otherwise on a slow heartbeat — enough to keep the
             * pane's "still 4m" moving and to tell a consumer this task is still
             * alive, without a storage write (and a wake for everyone subscribed)
             * every single second. */
            if (changed || now - s_lastPublishUs >= kHeartbeatS * 1000000) {
                publishState();
                s_lastPublishUs = now;
            }
        }

        /* The 1 s cadence exists only to read the WoM latch; disabled or
         * absent, there is nothing to read and a wake per second just caps
         * every light-sleep nap. Park until a config change notifies. */
        itsPoll((s_present && s_enabled && s_armed) ? pdMS_TO_TICKS(kPollMs)
                                                    : portMAX_DELAY);
    }
}

void ImuService::onInit() {
    if (storageGetInt("s.imu.version", 0) < IMU_VERSION) {
        storageBegin();
        storageDefault("s.imu.enable", 1);          /* staged only where the part exists → on */
        storageDefault("s.imu.threshold_mg", 40);   /* a hand-carried device trips ~40 mg */
        storageDefault("s.imu.odr", 11);            /* 11 Hz ≈ 35 µA */
        storageDefault("s.imu.still_s", 30);
        storageSet("s.imu.version", IMU_VERSION);
        storageEnd();
    }
    cliRegisterCmd("imu", cliImu);

    if (CONFIG_IMU_SPI_HOST < 0 || CONFIG_IMU_CS_PIN < 0) {
        dbg("no SPI host/CS configured (CONFIG_IMU_*) — dormant");
        storageSet("imu.present", 0);
        storageSet("imu.state", "not present");
        return;
    }
    s_present = imuBusOpen();
    storageSet("imu.present", s_present ? 1 : 0);
    storageSet("imu.model", s_present ? "QMI8658" : "");
    if (!s_present) {
        warn("no QMI8658 on SPI%d CS%d", CONFIG_IMU_SPI_HOST, CONFIG_IMU_CS_PIN);
        storageSet("imu.state", "not present");
        return;
    }
    info("QMI8658 present on SPI%d CS%d", CONFIG_IMU_SPI_HOST, CONFIG_IMU_CS_PIN);

    /* Boot counts as motion: something moved this device to switch it on, and a
     * consumer that parks on stillness should start from "awake" rather than
     * from a still-timer that has been running since the epoch. */
    s_lastMotionUs = esp_timer_get_time();
    s_moving       = true;

    imuIntInit();
    s_task = spawnTask(imuTaskMain, TAG, 4096, nullptr, 1, 0, STACK_PSRAM);
}
