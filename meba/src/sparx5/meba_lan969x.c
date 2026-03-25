// Copyright (c) 2004-2020 Microchip Technology Inc. and its subsidiaries.
// SPDX-License-Identifier: MIT

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <dirent.h>
#include <string.h>

#include <microchip/ethernet/board/api.h>

#include "meba_aux.h"
#include "meba_generic.h"
#include "meba_common.h"

#define VTSS_GPIOS_MAX        67
#define VTSS_MSLEEP(m)        usleep((m) * 1000)
#define VTSS_TS_IO_ARRAY_SIZE 8 // Laguna has 8 pins compared to 4 on FireAnt.

#define INDYPHY_INTERRUPT 11

/* Number of ports with 24V power control via 74HC595 shift registers */
#define PORT_POWER_COUNT 16

/* Local mapping table */
typedef struct {
    int32_t                chip_port;
    mesa_miim_controller_t miim_controller;
    uint8_t                miim_addr;
    mesa_port_interface_t  mac_if;
    meba_port_cap_t        cap;
    mesa_internal_bw_t     max_bw;
    uint8_t                sgpio_port;
    uint8_t                i2c_port;
    mesa_bool_t            ts_phy;
    mesa_bool_t            poe_support;
    uint8_t                poe_port;
} port_map_t;

static const meba_ptp_rs422_conf_t pcb8398_rs422_conf = {
    .gpio_rs422_1588_mstoen = -1,
    .gpio_rs422_1588_slvoen = -1,
    .ptp_pin_ldst = 5,
    .ptp_pin_ppso = 4,
    .ptp_rs422_pps_int_id = MEBA_EVENT_PTP_PIN_4,
    .ptp_rs422_ldsv_int_id = MEBA_EVENT_PTP_PIN_5,
    .serial_port = "/dev/ttyAT1"};

static const meba_event_t init_int_source_id[VTSS_TS_IO_ARRAY_SIZE] = {
    MEBA_EVENT_PTP_PIN_0, MEBA_EVENT_PTP_PIN_1, MEBA_EVENT_PTP_PIN_2, MEBA_EVENT_PTP_PIN_3,
    MEBA_EVENT_PTP_PIN_4, MEBA_EVENT_PTP_PIN_5, MEBA_EVENT_PTP_PIN_5, MEBA_EVENT_PTP_PIN_5};

static const uint32_t pin_conf_pcb8398[VTSS_TS_IO_ARRAY_SIZE] = {
    (MEBA_PTP_IO_CAP_UNUSED),
    (MEBA_PTP_IO_CAP_UNUSED),
    (MEBA_PTP_IO_CAP_UNUSED),
    (MEBA_PTP_IO_CAP_UNUSED),
    (MEBA_PTP_IO_CAP_UNUSED),
    (MEBA_PTP_IO_CAP_UNUSED),
    (MEBA_PTP_IO_CAP_UNUSED),
    (MEBA_PTP_IO_CAP_UNUSED)};

#define LAGUNA_CAP_SFP                                                                             \
    (MEBA_PORT_CAP_SD_ENABLE | MEBA_PORT_CAP_SD_HIGH | MEBA_PORT_CAP_SD_INTERNAL |                 \
     MEBA_PORT_CAP_SFP_DETECT | MEBA_PORT_CAP_SFP_ONLY)
#define LAGUNA_CAP_10G_FDX                                                                         \
    (MEBA_PORT_CAP_10G_FDX | MEBA_PORT_CAP_5G_FDX | MEBA_PORT_CAP_SFP_2_5G |                       \
     MEBA_PORT_CAP_FLOW_CTRL | LAGUNA_CAP_SFP)

typedef enum { SFP_DETECT, SFP_FAULT, SFP_LOS } sfp_signal_t;

typedef struct {
    mesa_bool_t valid;
    uint8_t     gpio_moddet;
    uint8_t     gpio_txfault;
    uint8_t     gpio_los;
    uint8_t     gpio_txdis;
    uint8_t     gpio_rs;
} sfp_gpio_map_t;

/* Custom PCB8398 wiring:
 * - SFP1 diag on GPIO 61..66
 * - SFP2 diag on GPIO 36..41
 * Map logical ports 26/27 to these two cages. */
static const sfp_gpio_map_t pcb8398_sfp_gpio_map[30] = {
    [26] = {.valid = TRUE, .gpio_moddet = 64, .gpio_txfault = 62,
            .gpio_los = 65, .gpio_txdis = 63, .gpio_rs = 66},
    [27] = {.valid = TRUE, .gpio_moddet = 39, .gpio_txfault = 37,
            .gpio_los = 40, .gpio_txdis = 38, .gpio_rs = 41},
};

static const sfp_gpio_map_t *pcb8398_sfp_gpio_map_get(meba_inst_t inst,
                                                       mesa_port_no_t port_no)
{
    meba_board_state_t *board = INST2BOARD(inst);
    uint32_t            chip_port;

    if (port_no >= board->port_cnt) {
        return NULL;
    }

    chip_port = board->port[port_no].map.map.chip_port;
    if (chip_port >= (sizeof(pcb8398_sfp_gpio_map) / sizeof(pcb8398_sfp_gpio_map[0]))) {
        return NULL;
    }

    if (!pcb8398_sfp_gpio_map[chip_port].valid) {
        return NULL;
    }

    return &pcb8398_sfp_gpio_map[chip_port];
}

static mesa_bool_t pcb8398_sfp_gpio_get(meba_inst_t inst, mesa_port_no_t port_no, sfp_signal_t sfp,
                                        mesa_bool_t *value)
{
    const sfp_gpio_map_t *map = pcb8398_sfp_gpio_map_get(inst, port_no);
    mesa_bool_t         v = 0;
    uint8_t             gpio_no;

    if (value == NULL || map == NULL) {
        return FALSE;
    }

    if (sfp == SFP_DETECT) {
        gpio_no = map->gpio_moddet;
    } else if (sfp == SFP_FAULT) {
        gpio_no = map->gpio_txfault;
    } else {
        gpio_no = map->gpio_los;
    }

    if (mesa_gpio_read(NULL, 0, gpio_no, &v) != MESA_RC_OK) {
        return FALSE;
    }

    if (sfp == SFP_DETECT) {
        *value = (v ? FALSE : TRUE); /* MODDET is active low */
    } else {
        *value = (v ? TRUE : FALSE); /* TXFAULT/LOS active high */
    }

    return TRUE;
}

static mesa_bool_t pcb8398_sfp_gpio_has_port(meba_inst_t inst, mesa_port_no_t port_no)
{
    return pcb8398_sfp_gpio_map_get(inst, port_no) != NULL;
}

/* ============================================================================
 * Port Power Control via 74HC595 shift registers (Linux gpiolib sysfs)
 *
 * The 74HC595 GPIO expander is exposed as a gpiochip in /sys/class/gpio/.
 * We find its base GPIO number by scanning for label "74hc595", then access
 * individual port power GPIOs via sysfs.
 * ============================================================================
 */

/**
 * Find the base GPIO number of the 74HC595 gpiochip.
 * Returns the base number, or -1 if not found.
 */
static int pcb8398_port_power_find_gpiochip(meba_inst_t inst)
{
    DIR           *dp;
    struct dirent *ep;
    int            base = -1;

    dp = opendir("/sys/class/gpio/");
    if (dp == NULL) {
        T_I(inst, "Cannot open /sys/class/gpio/");
        return -1;
    }

    while ((ep = readdir(dp)) != NULL) {
        if (strncmp(ep->d_name, "gpiochip", 8) != 0) {
            continue;
        }

        char label_path[512];
        snprintf(label_path, sizeof(label_path), "/sys/class/gpio/%s/label", ep->d_name);

        int fd = open(label_path, O_RDONLY);
        if (fd < 0) {
            continue;
        }

        char label[64] = {0};
        ssize_t n = read(fd, label, sizeof(label) - 1);
        close(fd);

        if (n <= 0) {
            continue;
        }

        /* Remove trailing newline */
        if (n > 0 && label[n - 1] == '\n') {
            label[n - 1] = '\0';
        }

        if (strstr(label, "74hc595") != NULL || strstr(label, "74x164") != NULL) {
            /* Found it - read base number */
            char base_path[512];
            snprintf(base_path, sizeof(base_path), "/sys/class/gpio/%s/base", ep->d_name);

            fd = open(base_path, O_RDONLY);
            if (fd >= 0) {
                char buf[32] = {0};
                if (read(fd, buf, sizeof(buf) - 1) > 0) {
                    base = atoi(buf);
                }
                close(fd);
            }
            T_I(inst, "Found 74HC595 port power gpiochip: %s, base=%d", ep->d_name, base);
            break;
        }
    }

    closedir(dp);
    return base;
}

/**
 * Export a GPIO to sysfs if not already exported.
 */
static mesa_bool_t pcb8398_port_power_gpio_export(meba_inst_t inst, int gpio_num)
{
    char gpio_path[64];
    snprintf(gpio_path, sizeof(gpio_path), "/sys/class/gpio/gpio%d", gpio_num);

    /* Check if already exported */
    if (access(gpio_path, F_OK) == 0) {
        return TRUE;
    }

    /* Export it */
    int fd = open("/sys/class/gpio/export", O_WRONLY);
    if (fd < 0) {
        T_E(inst, "Cannot open /sys/class/gpio/export");
        return FALSE;
    }

    char buf[16];
    int len = snprintf(buf, sizeof(buf), "%d", gpio_num);
    ssize_t written = write(fd, buf, len);
    close(fd);

    if (written != len) {
        T_E(inst, "Failed to export GPIO %d", gpio_num);
        return FALSE;
    }

    /* Brief delay for sysfs to create the node */
    usleep(10000);

    /* Set direction to output (if direction file exists).
     * Output-only expanders like 74HC595 may not have a direction file
     * since they are inherently output-only. */
    snprintf(gpio_path, sizeof(gpio_path), "/sys/class/gpio/gpio%d/direction", gpio_num);
    fd = open(gpio_path, O_WRONLY);
    if (fd >= 0) {
        written = write(fd, "out", 3);
        close(fd);
        if (written != 3) {
            T_E(inst, "Failed to set GPIO %d direction", gpio_num);
            return FALSE;
        }
    }
    /* If direction file doesn't exist, that's OK for output-only expanders */

    return TRUE;
}

/**
 * Set a port power GPIO value.
 */
static mesa_bool_t pcb8398_port_power_gpio_set(meba_inst_t inst, int gpio_num, mesa_bool_t value)
{
    char path[64];
    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/value", gpio_num);

    int fd = open(path, O_WRONLY);
    if (fd < 0) {
        T_E(inst, "Cannot open %s for writing", path);
        return FALSE;
    }

    const char *val_str = value ? "1" : "0";
    ssize_t written = write(fd, val_str, 1);
    close(fd);

    T_D(inst, "GPIO %d: wrote '%s', result=%zd", gpio_num, val_str, written);
    return (written == 1);
}

/**
 * Read a port power GPIO value.
 */
static mesa_bool_t pcb8398_port_power_gpio_get(meba_inst_t inst, int gpio_num, mesa_bool_t *value)
{
    char path[64];
    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/value", gpio_num);

    int fd = open(path, O_RDONLY);
    if (fd < 0) {
        T_E(inst, "Cannot open %s for reading", path);
        return FALSE;
    }

    char buf[4] = {0};
    ssize_t bytes_read = read(fd, buf, sizeof(buf) - 1);
    close(fd);

    if (bytes_read <= 0) {
        T_E(inst, "Failed to read GPIO %d value", gpio_num);
        return FALSE;
    }

    *value = (buf[0] == '1') ? TRUE : FALSE;
    T_D(inst, "GPIO %d: read '%c' -> %s", gpio_num, buf[0], *value ? "ON" : "OFF");
    return TRUE;
}

/**
 * Initialize port power control: find gpiochip and export all GPIOs.
 */
static void pcb8398_port_power_init(meba_inst_t inst)
{
    meba_board_state_t *board = INST2BOARD(inst);

    board->port_power_gpio_base = -1;
    board->port_power_state = 0;

    int base = pcb8398_port_power_find_gpiochip(inst);
    if (base < 0) {
        T_I(inst, "74HC595 port power gpiochip not found - power control unavailable");
        return;
    }

    board->port_power_gpio_base = base;

    /* Export all 16 GPIOs and set initial state to OFF */
    for (int i = 0; i < PORT_POWER_COUNT; i++) {
        if (!pcb8398_port_power_gpio_export(inst, base + i)) {
            T_E(inst, "Failed to export port power GPIO %d", i);
        }
        /* Set initial state to OFF */
        pcb8398_port_power_gpio_set(inst, base + i, FALSE);
    }

    T_I(inst, "Port power control initialized with gpio base %d", base);
}

/**
 * Set port power state (MEBA API wrapper).
 * port_no: logical port number (0-15 for port power control)
 * enable: TRUE to enable 24V power, FALSE to disable
 * Returns: MESA_RC_OK on success,
 *          MESA_RC_NOT_IMPLEMENTED if port power control not available,
 *          MESA_RC_ERROR on failure.
 */
static mesa_rc lan969x_port_power_set(meba_inst_t inst, mesa_port_no_t port_no, mesa_bool_t enable)
{
    meba_board_state_t *board = INST2BOARD(inst);

    if (board->port_power_gpio_base < 0) {
        /* Port power control not available (74HC595 gpiochip not found) */
        return MESA_RC_NOT_IMPLEMENTED;
    }

    /* Only the first 16 ports have power control */
    if (port_no >= PORT_POWER_COUNT) {
        return MESA_RC_OK;  /* Not an error, just no power control for this port */
    }

    int gpio_num = board->port_power_gpio_base + port_no;

    if (!pcb8398_port_power_gpio_set(inst, gpio_num, enable)) {
        T_E(inst, "Failed to set port %u power to %s", port_no, enable ? "ON" : "OFF");
        return MESA_RC_ERROR;
    }

    /* Update state tracking */
    if (enable) {
        board->port_power_state |= (1U << port_no);
    } else {
        board->port_power_state &= ~(1U << port_no);
    }

    T_I(inst, "Port %u power %s (gpio %d)", port_no, enable ? "ON" : "OFF", gpio_num);
    return MESA_RC_OK;
}

/**
 * Get port power state (MEBA API).
 * port_no: logical port number (0-15 for port power control)
 * enabled: [OUT] current power state
 * Returns: MESA_RC_OK on success,
 *          MESA_RC_NOT_IMPLEMENTED if port power control not available,
 *          MESA_RC_ERROR on failure.
 */
static mesa_rc lan969x_port_power_get(meba_inst_t inst, mesa_port_no_t port_no, mesa_bool_t *enabled)
{
    meba_board_state_t *board = INST2BOARD(inst);

    if (enabled == NULL) {
        return MESA_RC_ERROR;
    }

    if (board->port_power_gpio_base < 0) {
        /* Port power control not available (74HC595 gpiochip not found) */
        return MESA_RC_NOT_IMPLEMENTED;
    }

    /* Only the first 16 ports have power control */
    if (port_no >= PORT_POWER_COUNT) {
        *enabled = FALSE;
        return MESA_RC_OK;
    }

    int gpio_num = board->port_power_gpio_base + port_no;

    /* Read actual GPIO state from sysfs */
    if (!pcb8398_port_power_gpio_get(inst, gpio_num, enabled)) {
        /* Fall back to cached state */
        *enabled = (board->port_power_state & (1U << port_no)) ? TRUE : FALSE;
        T_D(inst, "Port %u: using cached state: %s", port_no, *enabled ? "ON" : "OFF");
    }

    return MESA_RC_OK;
}

static port_map_t *meba_port_map = NULL;

static port_map_t port_table_pcb8398[] = {
    /* Physical ports 1-4: chip_port 16-19 (QSGMII group C, MIIM addr 0-3) */
    {16, MESA_MIIM_CONTROLLER_0,    0,  MESA_PORT_INTERFACE_QSGMII,     MEBA_PORT_CAP_TRI_SPEED_COPPER,
    MESA_BW_1G,                                                                                                     0,  0, 1, 0, 0 },
    {17, MESA_MIIM_CONTROLLER_0,    1,  MESA_PORT_INTERFACE_QSGMII,     MEBA_PORT_CAP_TRI_SPEED_COPPER,
    MESA_BW_1G,                                                                                                     0,  0, 1, 0, 0 },
    {18, MESA_MIIM_CONTROLLER_0,    2,  MESA_PORT_INTERFACE_QSGMII,     MEBA_PORT_CAP_TRI_SPEED_COPPER,
    MESA_BW_1G,                                                                                                     0,  0, 1, 0, 0 },
    {19, MESA_MIIM_CONTROLLER_0,    3,  MESA_PORT_INTERFACE_QSGMII,     MEBA_PORT_CAP_TRI_SPEED_COPPER,
    MESA_BW_1G,                                                                                                     0,  0, 1, 0, 0 },
    /* Physical ports 5-8: chip_port 8-11 (QSGMII group A, MIIM addr 8-11) */
    {8,  MESA_MIIM_CONTROLLER_0,    8,  MESA_PORT_INTERFACE_QSGMII,     MEBA_PORT_CAP_TRI_SPEED_COPPER,
    MESA_BW_1G,                                                                                                     0,  0, 1, 0, 0 },
    {9,  MESA_MIIM_CONTROLLER_0,    9,  MESA_PORT_INTERFACE_QSGMII,     MEBA_PORT_CAP_TRI_SPEED_COPPER,
    MESA_BW_1G,                                                                                                     0,  0, 1, 0, 0 },
    {10, MESA_MIIM_CONTROLLER_0,    10, MESA_PORT_INTERFACE_QSGMII,     MEBA_PORT_CAP_TRI_SPEED_COPPER,
    MESA_BW_1G,                                                                                                     0,  0, 1, 0, 0 },
    {11, MESA_MIIM_CONTROLLER_0,    11, MESA_PORT_INTERFACE_QSGMII,     MEBA_PORT_CAP_TRI_SPEED_COPPER,
    MESA_BW_1G,                                                                                                     0,  0, 1, 0, 0 },
    /* Physical ports 9-12: chip_port 12-15 (QSGMII group B, MIIM addr 16-19) */
    {12, MESA_MIIM_CONTROLLER_0,    16, MESA_PORT_INTERFACE_QSGMII,     MEBA_PORT_CAP_TRI_SPEED_COPPER,
    MESA_BW_1G,                                                                                                     0,  0, 1, 0, 0 },
    {13, MESA_MIIM_CONTROLLER_0,    17, MESA_PORT_INTERFACE_QSGMII,     MEBA_PORT_CAP_TRI_SPEED_COPPER,
    MESA_BW_1G,                                                                                                     0,  0, 1, 0, 0 },
    {14, MESA_MIIM_CONTROLLER_0,    18, MESA_PORT_INTERFACE_QSGMII,     MEBA_PORT_CAP_TRI_SPEED_COPPER,
    MESA_BW_1G,                                                                                                     0,  0, 1, 0, 0 },
    {15, MESA_MIIM_CONTROLLER_0,    19, MESA_PORT_INTERFACE_QSGMII,     MEBA_PORT_CAP_TRI_SPEED_COPPER,
    MESA_BW_1G,                                                                                                     0,  0, 1, 0, 0 },
    /* Physical ports 13-16: chip_port 20-23 (QSGMII group D, MIIM addr 24-27) */
    {20, MESA_MIIM_CONTROLLER_0,    24, MESA_PORT_INTERFACE_QSGMII,     MEBA_PORT_CAP_TRI_SPEED_COPPER,
    MESA_BW_1G,                                                                                                     0,  0, 1, 0, 0 },
    {21, MESA_MIIM_CONTROLLER_0,    25, MESA_PORT_INTERFACE_QSGMII,     MEBA_PORT_CAP_TRI_SPEED_COPPER,
    MESA_BW_1G,                                                                                                     0,  0, 1, 0, 0 },
    {22, MESA_MIIM_CONTROLLER_0,    26, MESA_PORT_INTERFACE_QSGMII,     MEBA_PORT_CAP_TRI_SPEED_COPPER,
    MESA_BW_1G,                                                                                                     0,  0, 1, 0, 0 },
    {23, MESA_MIIM_CONTROLLER_0,    27, MESA_PORT_INTERFACE_QSGMII,     MEBA_PORT_CAP_TRI_SPEED_COPPER,
    MESA_BW_1G,                                                                                                     0,  0, 1, 0, 0 },
    /* Physical ports 17-18: SFP (chip_port 26-27) */
    {26, MESA_MIIM_CONTROLLER_NONE, 0,  MESA_PORT_INTERFACE_SFI,        LAGUNA_CAP_10G_FDX,
    MESA_BW_10G,                                                                                                       255, 100, 0, 0, 0 },
    {27, MESA_MIIM_CONTROLLER_NONE, 0,  MESA_PORT_INTERFACE_SFI,        LAGUNA_CAP_10G_FDX,
    MESA_BW_10G,                                                                                                       255, 101, 0, 0, 0 },
};

#define PCB8398_GPIO_FUNC_INFO_SIZE 8
static const mesa_gpio_func_info_t pcb8398_gpio_func_info[PCB8398_GPIO_FUNC_INFO_SIZE] = {
    {.gpio_no = VTSS_GPIOS_MAX, .alt = MESA_GPIO_FUNC_ALT_0},
    {.gpio_no = VTSS_GPIOS_MAX, .alt = MESA_GPIO_FUNC_ALT_0},
    {.gpio_no = VTSS_GPIOS_MAX, .alt = MESA_GPIO_FUNC_ALT_0},
    {.gpio_no = VTSS_GPIOS_MAX, .alt = MESA_GPIO_FUNC_ALT_0},
    {.gpio_no = VTSS_GPIOS_MAX, .alt = MESA_GPIO_FUNC_ALT_0},
    {.gpio_no = VTSS_GPIOS_MAX, .alt = MESA_GPIO_FUNC_ALT_0},
    {.gpio_no = VTSS_GPIOS_MAX, .alt = MESA_GPIO_FUNC_ALT_0},
    {.gpio_no = VTSS_GPIOS_MAX, .alt = MESA_GPIO_FUNC_ALT_0},
};

static void port_entry_map(meba_port_entry_t *entry, port_map_t *map)
{
    entry->map.chip_port = map->chip_port;
    entry->map.miim_controller = map->miim_controller;
    entry->map.miim_addr = map->miim_addr;
    entry->mac_if = map->mac_if;
    entry->cap = map->cap;
    entry->map.max_bw = map->max_bw;
    entry->poe_support = map->poe_support;
    entry->poe_port = map->poe_port;
}

static void lan966x_init_port_table(meba_inst_t inst, int port_cnt, port_map_t *map)
{
    meba_board_state_t *board = INST2BOARD(inst);
    mesa_port_no_t      port_no;

    /* Fill out port mapping table */
    board->port_cnt = port_cnt;
    for (port_no = 0; port_no < port_cnt; port_no++) {
        port_entry_map(&board->port[port_no].map, &map[port_no]);
        board->port[port_no].ts_phy = map[port_no].ts_phy;
        if (map[port_no].mac_if == MESA_PORT_INTERFACE_QSGMII) {
            board->port[port_no].map.phy_base_port = (map[port_no].chip_port / 4) * 4;
        }
    }
}

static mesa_rc lan969x_board_init(meba_inst_t inst)
{
    meba_board_state_t *board = INST2BOARD(inst);
    uint32_t            gpio_no;

    /* Configure GPIOs for MIIM/MDIO bus 0 */
    for (gpio_no = 9; gpio_no <= 10; gpio_no++) {
        (void)mesa_gpio_mode_set(NULL, 0, gpio_no, MESA_GPIO_ALT_0);
    }

    /* Configure GPIO 11 as interrupt from PHYs */
    (void)mesa_gpio_mode_set(NULL, 0, INDYPHY_INTERRUPT, MESA_GPIO_IN_INT);

    /* PHY reset via GPIO 7 */
    gpio_no = 7;
    (void)mesa_gpio_mode_set(NULL, 0, gpio_no, MESA_GPIO_OUT);
    (void)mesa_gpio_write(NULL, 0, gpio_no, 0);
    (void)mesa_gpio_write(NULL, 0, gpio_no, 1);

    /* Configure SFP GPIOs */
    for (mesa_port_no_t p = 0; p < board->port_cnt; p++) {
        const sfp_gpio_map_t *map = pcb8398_sfp_gpio_map_get(inst, p);

        if (map == NULL) {
            continue;
        }

        (void)mesa_gpio_mode_set(NULL, 0, map->gpio_moddet, MESA_GPIO_IN);
        (void)mesa_gpio_mode_set(NULL, 0, map->gpio_txfault, MESA_GPIO_IN);
        (void)mesa_gpio_mode_set(NULL, 0, map->gpio_los, MESA_GPIO_IN);
        (void)mesa_gpio_mode_set(NULL, 0, map->gpio_txdis, MESA_GPIO_OUT);
        /* Deassert TX_DISABLE by default */
        (void)mesa_gpio_write(NULL, 0, map->gpio_txdis, 0);
        /* Assert rate select high for 10G SFP+ operation */
        (void)mesa_gpio_mode_set(NULL, 0, map->gpio_rs, MESA_GPIO_OUT);
        (void)mesa_gpio_write(NULL, 0, map->gpio_rs, 1);
    }

    return MESA_RC_OK;
}

static uint32_t lan969x_capability(meba_inst_t inst, int cap)
{
    meba_board_state_t *board = INST2BOARD(inst);
    T_N(inst, "Called - %d", cap);
    switch (cap) {
    case MEBA_CAP_POE:                         return 0;
    case MEBA_CAP_1588_CLK_ADJ_DAC:
    case MEBA_CAP_1588_REF_CLK_SEL:            return 0;
    case MEBA_CAP_TEMP_SENSORS:                return 1;
    case MEBA_CAP_BOARD_PORT_COUNT:
    case MEBA_CAP_BOARD_PORT_MAP_COUNT:        return board->port_cnt;
    case MEBA_CAP_LED_MODES:
    case MEBA_CAP_DYING_GASP:
    case MEBA_CAP_FAN_SUPPORT:                 return 0;
    case MEBA_CAP_LED_DIM_SUPPORT:
    case MEBA_CAP_BOARD_HAS_PCB107_CPLD:
    case MEBA_CAP_PCB107_CPLD_CS_VIA_MUX:
    case MEBA_CAP_BOARD_HAS_PCB135_CPLD:
    case MEBA_CAP_SYNCE_CLOCK_DPLL:
    case MEBA_CAP_SYNCE_CLOCK_OUTPUT_CNT:
    case MEBA_CAP_SYNCE_PTP_CLOCK_OUTPUT:
    case MEBA_CAP_SYNCE_HO_POST_FILTERING_BW:
    case MEBA_CAP_SYNCE_CLOCK_EEC_OPTION_CNT:  return 0;
    case MEBA_CAP_ONE_PPS_INT_ID:              return MEBA_EVENT_PTP_PIN_3;
    case MEBA_CAP_SYNCE_DPLL_MODE_SINGLE:
    case MEBA_CAP_SYNCE_STATION_CLOCK_MUX_SET:
    case MEBA_CAP_POE_BT:                      return 0;
    case MEBA_CAP_CPU_PORTS_COUNT:             return 0;
    case MEBA_CAP_RECOMMENDED_MTU_SIZE:        return 0;
    case MEBA_CAP_SYNCE_DPLL_MODE_DUAL:        return 0;

    default: T_E(inst, "Unknown capability %d", cap); MEBA_ASSERT(0);
    }
    return 0;
}

static mesa_rc lan969x_port_entry_get(meba_inst_t        inst,
                                      mesa_port_no_t     port_no,
                                      meba_port_entry_t *entry)
{
    mesa_rc             rc;
    meba_board_state_t *board = INST2BOARD(inst);

    T_N(inst, "Called");
    if (port_no < board->port_cnt) {
        *entry = board->port[port_no].map;
        rc = MESA_RC_OK;
    } else {
        rc = MESA_RC_ERROR;
    }
    T_N(inst, "Called(%d): rc %d - chip %d, miim bus %d, addr: %d", port_no, rc,
        entry->map.chip_port, entry->map.miim_controller, entry->map.miim_addr);
    return rc;
}

static mesa_rc lan969x_sfp_i2c_xfer(meba_inst_t    inst,
                                    mesa_port_no_t port_no,
                                    mesa_bool_t    write,
                                    uint8_t        i2c_addr,
                                    uint8_t        addr,
                                    uint8_t       *data,
                                    uint8_t        cnt,
                                    mesa_bool_t    word_access)
{
    mesa_rc rc = MESA_RC_ERROR;
    uint8_t i2c_port = meba_port_map[port_no].i2c_port;

    T_N(inst, "Called");

    if (write) { // cnt ignored
        uint8_t i2c_data[3];
        i2c_data[0] = addr;
        memcpy(&i2c_data[1], data, 2);
        rc = inst->iface.i2c_write(i2c_port, i2c_addr, i2c_data, 3);
    } else {
        rc = inst->iface.i2c_read(i2c_port, i2c_addr, addr, data, cnt);
    }

    T_D(inst, "i2c %s port %d - address 0x%02x:0x%02x, %d bytes return %d",
        write ? "write" : "read", i2c_port, i2c_addr, addr, cnt, rc);
    return rc;
}

// For backwards compatibility (use lan969x_sfp_status_get())
static mesa_rc lan969x_sfp_insertion_status_get(meba_inst_t inst, mesa_port_list_t *present)
{
    meba_board_state_t    *board = INST2BOARD(inst);

    T_N(inst, "Called");
    mesa_port_list_clear(present);

    for (mesa_port_no_t port_no = 0; port_no < board->port_cnt; port_no++) {
        mesa_bool_t detect = FALSE;

        if (!is_sfp_port(board->port[port_no].map.cap) ||
            !pcb8398_sfp_gpio_has_port(inst, port_no)) {
            continue;
        }
        if (pcb8398_sfp_gpio_get(inst, port_no, SFP_DETECT, &detect)) {
            mesa_port_list_set(present, port_no, detect);
        }
    }
    return MESA_RC_OK;
}

static mesa_rc lan969x_sfp_status_get(meba_inst_t        inst,
                                      mesa_port_no_t     port_no,
                                      meba_sfp_status_t *status)
{
    meba_board_state_t    *board = INST2BOARD(inst);

    if (!is_sfp_port(board->port[port_no].map.cap)) {
        return MESA_RC_OK;
    }

    if (pcb8398_sfp_gpio_has_port(inst, port_no)) {
        mesa_bool_t v;

        if (pcb8398_sfp_gpio_get(inst, port_no, SFP_LOS, &v)) {
            status->los = v;
        }
        if (pcb8398_sfp_gpio_get(inst, port_no, SFP_DETECT, &v)) {
            status->present = v;
        }
        if (pcb8398_sfp_gpio_get(inst, port_no, SFP_FAULT, &v)) {
            status->tx_fault = v;
        }
    } else {
        status->present = FALSE;
        status->los = TRUE;
        status->tx_fault = TRUE;
    }

    return MESA_RC_OK;
}

// Applies only to SFPs where TxDisable is enabled/disabled
static mesa_rc lan969x_port_admin_state_set(meba_inst_t                    inst,
                                            mesa_port_no_t                 port_no,
                                            const meba_port_admin_state_t *state)
{
    const sfp_gpio_map_t *map = pcb8398_sfp_gpio_map_get(inst, port_no);

    if (map == NULL) {
        return MESA_RC_OK;
    }

    /* TX_DISABLE is active high on the direct GPIO wiring. */
    (void)mesa_gpio_write(NULL, 0, map->gpio_txdis,
                          state->enable ? FALSE : TRUE);
    return MESA_RC_OK;
}

static mesa_rc lan969x_port_led_update(meba_inst_t                    inst,
                                       mesa_port_no_t                 port_no,
                                       const mesa_port_status_t      *status,
                                       const mesa_port_counters_t    *counters,
                                       const meba_port_admin_state_t *state)
{
    return MESA_RC_OK;
}

static mesa_rc lan969x_status_led_set(meba_inst_t      inst,
                                      meba_led_type_t  type,
                                      meba_led_color_t color)
{
    return MESA_RC_OK;
}

static mesa_rc lan969x_reset(meba_inst_t inst, meba_reset_point_t reset)
{
    meba_board_state_t *board = INST2BOARD(inst);
    mesa_rc             rc = MESA_RC_OK;

    T_D(inst, "Called - %d", reset);
    switch (reset) {
    case MEBA_BOARD_INITIALIZE:      lan969x_board_init(inst); break;
    case MEBA_PORT_RESET:            break;
    case MEBA_STATUS_LED_INITIALIZE: break;
    case MEBA_PORT_LED_INITIALIZE:   break;
    case MEBA_PORT_RESET_POST:       break;
    case MEBA_FAN_INITIALIZE:       break;
    case MEBA_SENSOR_INITIALIZE:    break;
    case MEBA_INTERRUPT_INITIALIZE: break;
    case MEBA_POE_INITIALIZE:       break;
    case MEBA_PHY_INITIALIZE:
        inst->phy_devices = (mepa_device_t **)&board->phy_devices;
        inst->phy_device_cnt = board->port_cnt;
        meba_phy_driver_init(inst);
        break;
    default: rc = MESA_RC_ERROR;
    }
    T_D(inst, "Called - %d - Done", reset);
    return rc;
}

static mesa_rc sgpio_handler(meba_inst_t         inst,
                             meba_board_state_t *board,
                             meba_event_signal_t signal_notifier)
{
    return MESA_RC_ERROR;
}

// IRQ Support
static mesa_rc lan969x_event_enable(meba_inst_t inst, meba_event_t event_id, mesa_bool_t enable)
{
    mesa_rc               rc = MESA_RC_OK;
    meba_board_state_t   *board = INST2BOARD(inst);
    mesa_port_no_t        port_no;
    mesa_ptp_event_type_t ptp_event;

    T_D(inst, "%sable event %d", enable ? "en" : "dis", event_id);

    switch (event_id) {
    case MEBA_EVENT_SYNC:
    case MEBA_EVENT_EXT_SYNC:
    case MEBA_EVENT_EXT_1_SYNC:
    case MEBA_EVENT_CLK_ADJ:
    case MEBA_EVENT_VOE:        return rc; // Dummy for now

    case MEBA_EVENT_LOS:
        for (port_no = 0; port_no < board->port_cnt; port_no++) {
            const sfp_gpio_map_t *map = pcb8398_sfp_gpio_map_get(inst, port_no);

            if (!pcb8398_sfp_gpio_has_port(inst, port_no)) {
                continue;
            }
            (void)mesa_gpio_event_enable(NULL, 0, map->gpio_los, enable);
            (void)mesa_gpio_event_enable(NULL, 0, map->gpio_txfault, enable);
            (void)mesa_gpio_event_enable(NULL, 0, map->gpio_moddet, enable);
        }
        for (port_no = 0; port_no < board->port_cnt; port_no++) {
            if (is_phy_port(board->port[port_no].map.cap)) {
                if ((rc = meba_phy_event_enable_set(inst, port_no, MEPA_LINK_LOS, enable)) !=
                    MESA_RC_OK) {
                    T_E(inst, "Could not enable MEPA_LINK_LOS in phy (%d)", port_no);
                }
            }
        }
        break;
    case MEBA_EVENT_FLNK:
        for (port_no = 0; port_no < board->port_cnt - 1; port_no++) {
            if (is_phy_port(board->port[port_no].map.cap)) {
                if ((rc = meba_phy_event_enable_set(inst, port_no, VTSS_PHY_LINK_FFAIL_EV,
                                                    enable)) != MESA_RC_OK) {
                    T_E(inst, "Could not enable VTSS_PHY_LINK_FFAIL_EV in phy (%d)", port_no);
                }
            }
        }
        break;
    case MEBA_EVENT_PTP_PIN_0:
    case MEBA_EVENT_PTP_PIN_1:
    case MEBA_EVENT_PTP_PIN_2:
    case MEBA_EVENT_PTP_PIN_3:
    case MEBA_EVENT_PTP_PIN_4:
    case MEBA_EVENT_PTP_PIN_5:
    case MEBA_EVENT_CLK_TSTAMP:
        ptp_event = meba_generic_ptp_source_to_event(inst, event_id);
        if ((rc = mesa_ptp_event_enable(NULL, ptp_event, enable)) != MESA_RC_OK) {
            T_E(inst, "mesa_ptp_event_enable = %d", rc);
        }
        break;

    case MEBA_EVENT_INGR_ENGINE_ERR:
    case MEBA_EVENT_INGR_RW_PREAM_ERR:
    case MEBA_EVENT_INGR_RW_FCS_ERR:
    case MEBA_EVENT_EGR_ENGINE_ERR:
    case MEBA_EVENT_EGR_RW_FCS_ERR:
    case MEBA_EVENT_EGR_TIMESTAMP_CAPTURED:
    case MEBA_EVENT_EGR_FIFO_OVERFLOW:      {
        mepa_ts_event_t event = meba_generic_phy_ts_source_to_event(inst, event_id);
        for (port_no = 0; port_no < board->port_cnt; port_no++) {
            if (board->port[port_no].ts_phy &&
                (rc = meba_phy_ts_event_set(inst, port_no, enable, event)) != MESA_RC_OK) {
                T_E(inst, "vtss_phy_ts_event_enable_set(%d, %d, %d) = %d", port_no, enable, event,
                    rc);
            }
        }
    } break;
    case MEBA_EVENT_KR:
        // Handled in kr application
        break;

    default:
        rc = MESA_RC_NOT_IMPLEMENTED; // Will occur as part of probing
        break;
    }

    return rc;
}

static mesa_rc ext0_handler(meba_inst_t         inst,
                            meba_board_state_t *board,
                            meba_event_signal_t signal_notifier)
{
    int            handled = 0;
    mesa_port_no_t port_no;
    for (port_no = 0; port_no < board->port_cnt; port_no++) {
        if (is_phy_port(board->port[port_no].map.cap)) {
            if (meba_generic_phy_event_check(inst, port_no, signal_notifier) == MESA_RC_OK) {
                T_D(inst, "port(%d) PHY IRQ handled", port_no);
                handled++;
            }
        }
    }
    return handled ? MESA_RC_OK : MESA_RC_ERROR;
}

static mesa_rc phy_interrupt_handler(meba_inst_t         inst,
                                     meba_board_state_t *board,
                                     meba_event_signal_t signal_notifier)
{
    mesa_port_no_t port_no;
    int            handled = 0;

    for (port_no = 0; port_no < board->port_cnt - 1; port_no++) {
        if (is_phy_port(board->port[port_no].map.cap)) {
            // Check for Cu Phy events
            if (meba_generic_phy_event_check(inst, port_no, signal_notifier) == MESA_RC_OK) {
                handled++;
            }
            if (meba_generic_phy_timestamp_check(inst, port_no, signal_notifier) == MESA_RC_OK) {
                handled++;
            }
        }
    }
    return handled ? MESA_RC_OK : MESA_RC_ERROR;
}

static mesa_rc gpio_handler(meba_inst_t         inst,
                            meba_board_state_t *board,
                            meba_event_signal_t signal_notifier)
{
    int         gpio_cnt = MESA_CAP(MESA_CAP_MISC_GPIO_CNT);
    mesa_bool_t gpio_events[gpio_cnt];
    mesa_rc     rc;
    int         handled = 0;

    if ((rc = mesa_gpio_event_poll(NULL, 0, gpio_events)) != MESA_RC_OK) {
        T_E(inst, "mesa_gpio_event_poll: %d", rc);
        return rc;
    }

    if (gpio_events[2]) {
        if ((rc = mesa_gpio_event_enable(NULL, 0, 2, false)) != MESA_RC_OK) {
            T_E(inst, "mesa_gpio_event_enable = %d", rc);
        }
        signal_notifier(MEBA_EVENT_PUSH_BUTTON, 0);
        handled++;
    }

    for (mesa_port_no_t p = 0; p < board->port_cnt; p++) {
        const sfp_gpio_map_t *map = pcb8398_sfp_gpio_map_get(inst, p);

        if (!pcb8398_sfp_gpio_has_port(inst, p)) {
            continue;
        }

        if (gpio_events[map->gpio_los] ||
            gpio_events[map->gpio_txfault] ||
            gpio_events[map->gpio_moddet]) {
            signal_notifier(MEBA_EVENT_LOS, p);
            handled++;
        }
    }

    if (gpio_events[INDYPHY_INTERRUPT]) {
        mesa_bool_t gpio_state;
        while (MESA_RC_OK == mesa_gpio_read(NULL, 0, INDYPHY_INTERRUPT, &gpio_state) &&
               gpio_state == 0) {
            T_I(inst, "Got interrupt from gpio #%u value: %d", INDYPHY_INTERRUPT, gpio_state);
            if ((rc = phy_interrupt_handler(inst, board, signal_notifier)) == MESA_RC_OK) {
                handled++;
            }
        }
    }

    return handled ? MESA_RC_OK : MESA_RC_ERROR;
}

static mesa_rc kr_irq2port(meba_inst_t inst, mesa_irq_t chip_irq, mesa_port_no_t *port_no)
{
    meba_board_state_t *board = INST2BOARD(inst);
    mesa_rc             rc = MESA_RC_ERROR;
    uint32_t            chip_port = 0;

    // Convert chip IRQs to to chip ports
    // and check if the chip port exists in the port map.
    switch (chip_irq) {
    case MESA_IRQ_KR_SD10G_0: chip_port = 0; break;
    case MESA_IRQ_KR_SD10G_1: chip_port = 4; break;
    case MESA_IRQ_KR_SD10G_2: chip_port = 8; break;
    case MESA_IRQ_KR_SD10G_3: chip_port = 12; break;
    case MESA_IRQ_KR_SD10G_4: chip_port = 16; break;
    case MESA_IRQ_KR_SD10G_5: chip_port = 20; break;
    case MESA_IRQ_KR_SD10G_6: chip_port = 24; break;
    case MESA_IRQ_KR_SD10G_7: chip_port = 25; break;
    case MESA_IRQ_KR_SD10G_8: chip_port = 26; break;
    case MESA_IRQ_KR_SD10G_9: chip_port = 27; break;

    default: rc = MESA_RC_ERROR;
    }

    for (mesa_port_no_t p = 0; p < board->port_cnt; p++) {
        if (board->port[p].map.map.chip_port == chip_port) {
            *port_no = p;
            return MESA_RC_OK;
        }
    }

    return rc;
}

static mesa_rc kr_handler(meba_inst_t         inst,
                          meba_board_state_t *board,
                          mesa_irq_t          chip_irq,
                          meba_event_signal_t signal_notifier)
{
    mesa_port_no_t port_no = 0;
    if (kr_irq2port(inst, chip_irq, &port_no) != MESA_RC_OK) {
        return MESA_RC_OK; // Not used in the current board config
    }

    signal_notifier(MEBA_EVENT_KR, port_no);
    return MESA_RC_OK;
}

static mesa_rc lan969x_irq_handler(meba_inst_t         inst,
                                   mesa_irq_t          chip_irq,
                                   meba_event_signal_t signal_notifier)
{
    meba_board_state_t *board = INST2BOARD(inst);

    T_D(inst, "Called - irq %d", chip_irq);
    switch (chip_irq) {
    case MESA_IRQ_PTP_SYNC:   return meba_generic_ptp_handler(inst, signal_notifier);
    case MESA_IRQ_PTP_RDY:    signal_notifier(MEBA_EVENT_CLK_TSTAMP, 0); return MESA_RC_OK;
    case MESA_IRQ_OAM:        signal_notifier(MEBA_EVENT_VOE, 0); return MESA_RC_OK;
    case MESA_IRQ_SGPIO:      return sgpio_handler(inst, board, signal_notifier);
    case MESA_IRQ_EXT0:       return ext0_handler(inst, board, signal_notifier);
    case MESA_IRQ_GPIO:       return gpio_handler(inst, board, signal_notifier); return MESA_RC_OK;
    case MESA_IRQ_KR_SD10G_0:
    case MESA_IRQ_KR_SD10G_1:
    case MESA_IRQ_KR_SD10G_2:
    case MESA_IRQ_KR_SD10G_3:
    case MESA_IRQ_KR_SD10G_4:
    case MESA_IRQ_KR_SD10G_5:
    case MESA_IRQ_KR_SD10G_6:
    case MESA_IRQ_KR_SD10G_7:
    case MESA_IRQ_KR_SD10G_8:
    case MESA_IRQ_KR_SD10G_9: return kr_handler(inst, board, chip_irq, signal_notifier);
    default:                  break;
    }
    return MESA_RC_NOT_IMPLEMENTED;
}

static mesa_rc lan969x_irq_requested(meba_inst_t inst, mesa_irq_t chip_irq)
{
    mesa_rc rc = MESA_RC_NOT_IMPLEMENTED;

    switch (chip_irq) {
    case MESA_IRQ_PTP_SYNC:
    case MESA_IRQ_PTP_RDY:
    case MESA_IRQ_OAM:
    case MESA_IRQ_SGPIO:
    case MESA_IRQ_EXT0:
    case MESA_IRQ_GPIO:
    case MESA_IRQ_KR_SD10G_0:
    case MESA_IRQ_KR_SD10G_1:
    case MESA_IRQ_KR_SD10G_2:
    case MESA_IRQ_KR_SD10G_3:
    case MESA_IRQ_KR_SD10G_4:
    case MESA_IRQ_KR_SD10G_5:
    case MESA_IRQ_KR_SD10G_6:
    case MESA_IRQ_KR_SD10G_7:
    case MESA_IRQ_KR_SD10G_8:
    case MESA_IRQ_KR_SD10G_9: rc = MESA_RC_OK; break;
    default:                  break;
    }
    return rc;
}

static mesa_rc lan969x_ptp_rs422_conf_get(meba_inst_t inst, meba_ptp_rs422_conf_t *conf)
{
    T_N(inst, "Called");
    *conf = pcb8398_rs422_conf;
    return VTSS_RC_OK;
}

static mesa_rc lan969x_ptp_external_io_conf_get(meba_inst_t              inst,
                                                uint32_t                 io_pin,
                                                meba_ptp_io_cap_t *const board_assignment,
                                                meba_event_t *const      source_id)
{
    if (io_pin >= VTSS_TS_IO_ARRAY_SIZE) {
        return MESA_RC_ERROR;
    }
    // default pin assignment.
    *board_assignment = pin_conf_pcb8398[io_pin];

    *source_id = init_int_source_id[io_pin];
    return MESA_RC_OK;
}

static mesa_rc lan969x_gpio_func_info_get(meba_inst_t            inst,
                                          mesa_gpio_func_t       gpio_func,
                                          mesa_gpio_func_info_t *info)
{
    if (gpio_func < PCB8398_GPIO_FUNC_INFO_SIZE) {
        *info = pcb8398_gpio_func_info[gpio_func];
    } else {
        T_E(inst, "Invalid gpio_func %u", gpio_func);
        return MESA_RC_ERROR;
    }
    return MESA_RC_OK;
}

static mesa_rc lan969x_sensor_get(meba_inst_t inst, meba_sensor_t type, int six, int *value)
{
    mesa_rc rc = MESA_RC_ERROR;
    int16_t temp = 0;

    T_N(inst, "Called %d:%d", type, six);

    if (type == MEBA_SENSOR_BOARD_TEMP ||
        type == MEBA_SENSOR_PORT_TEMP) { // Port/Phy temperature not available
        rc = mesa_temp_sensor_get(NULL, &temp);
    }
    if (rc == MESA_RC_OK) {
        T_N(inst, "Temp %d:%d = %d", type, six, temp);
        *value = temp;
    } else {
        T_N(inst, "Temp %d:%d = [not read:%d]", type, six, rc);
    }
    return rc;
}

meba_inst_t lan969x_initialize(meba_inst_t inst, const meba_board_interface_t *callouts)
{
    meba_board_state_t *board;
    int                 pcb, target, pcb_var, port_cnt = 0;

    board = INST2BOARD(inst);

    // Get the board pcb type from DT
    if (meba_conf_get_hex(inst, "pcb", &pcb) != MESA_RC_OK) {
        fprintf(stderr, "Could not read pcb id\n");
        goto error_out;
    }
    // Get the target (TSN/HSN/...) from DT
    if (meba_conf_get_hex(inst, "target", &target) != MESA_RC_OK) {
        fprintf(stderr, "Could not read target\n");
        goto error_out;
    }

    // Get the port count from uboot (if any)
    if (meba_conf_get_hex(inst, "pcb_var", &pcb_var) == MESA_RC_OK) {
        port_cnt = pcb_var;
    }

    board->type = (board_type_t)pcb;
    inst->props.board_type = board->type;
    inst->props.target = target;
    board->port = (fa_port_info_t *)calloc(30, sizeof(fa_port_info_t));
    if (board->port == NULL) {
        fprintf(stderr, "Port table malloc failure\n");
        goto error_out;
    }
    // Note: Do NOT force ref_freq here. Let it default to MESA_CORE_REF_CLK_DEFAULT
    // so MESA reads the hardware strapping (REFCLK_SEL). The original code assumed
    // all PCB8398/PCB8422 boards have a SyncE DPLL providing 25MHz, but custom boards
    // may use the 39MHz crystal directly. Forcing 25MHz with clk_sel=2 on boards
    // without the DPLL causes PLL lock failure and system hang.
    // SyncE capability is already dynamically detected via meba_synce_spi_if_get_dpll_type().

    switch (board->type) {
    case BOARD_TYPE_LAGUNA_PCB8398:
        if (port_cnt == 0) {
            port_cnt = sizeof(port_table_pcb8398) / sizeof(port_map_t);
        }
        lan966x_init_port_table(inst, port_cnt, port_table_pcb8398);
        meba_port_map = port_table_pcb8398;
        break;
    default: break;
    }

    /* Initialize 24V port power control (PCB8398 only) */
    pcb8398_port_power_init(inst);

    T_I(inst, "Board: %s, type %d, target %4x, mux %d, %d ports", inst->props.name, board->type,
        inst->props.target, inst->props.mux_mode, board->port_cnt);

    // Hook up board API functions
    T_D(inst, "Hooking up board API");
    inst->api.meba_capability = lan969x_capability;
    inst->api.meba_port_entry_get = lan969x_port_entry_get;
    inst->api.meba_reset = lan969x_reset;
    inst->api.meba_sensor_get = lan969x_sensor_get;
    inst->api.meba_sfp_i2c_xfer = lan969x_sfp_i2c_xfer;
    inst->api.meba_sfp_insertion_status_get = lan969x_sfp_insertion_status_get;
    inst->api.meba_sfp_status_get = lan969x_sfp_status_get;
    inst->api.meba_port_admin_state_set = lan969x_port_admin_state_set;
    inst->api.meba_port_power_set = lan969x_port_power_set;
    inst->api.meba_port_power_get = lan969x_port_power_get;
    inst->api.meba_port_led_update = lan969x_port_led_update;
    inst->api.meba_led_intensity_set = NULL;
    inst->api.meba_fan_param_get = NULL;
    inst->api.meba_fan_conf_get = NULL;
    inst->api.meba_status_led_set = lan969x_status_led_set;
    inst->api.meba_irq_handler = lan969x_irq_handler;
    inst->api.meba_irq_requested = lan969x_irq_requested;
    inst->api.meba_event_enable = lan969x_event_enable;
    inst->api.meba_deinitialize = NULL;
    inst->api.meba_ptp_rs422_conf_get = lan969x_ptp_rs422_conf_get;
    inst->api.meba_gpio_func_info_get = lan969x_gpio_func_info_get;
    inst->api_synce = meba_synce_get();
    inst->api_tod = meba_tod_get();
    inst->api_poe = meba_poe_get();
    inst->api_cpu_port = NULL;
    inst->api.meba_serdes_tap_get = NULL;
    inst->api.meba_ptp_external_io_conf_get = lan969x_ptp_external_io_conf_get;
    return inst;

error_out:
    free(inst);
    return NULL;
}
