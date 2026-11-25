/**
 * @file mcp2515.c
 * @brief MCP2515 driver. See mcp2515.h.
 */

#include "drivers/mcp2515.h"
#include "config.h"
#include "drivers/spi.h"
#include "stm32f4xx_hal.h"
#include <errno.h>
#include <string.h>

// instructions: the first byte after CS goes low
#define INS_RESET     0xC0
#define INS_READ      0x03
#define INS_WRITE     0x02
#define INS_BITMOD    0x05
#define INS_STATUS    0xA0 // b0 RX0IF, b1 RX1IF, b2/b4/b6 TXB0/1/2 TXREQ
#define INS_LOAD_TX0  0x40 // write from TXB0SIDH; TXB1 0x42, TXB2 0x44
#define INS_READ_RX0  0x90 // read from RXB0SIDH, clears RX0IF when CS goes high
#define INS_READ_RX1  0x94

#define REG_CANSTAT  0x0E
#define REG_CANCTRL  0x0F
#define REG_TEC      0x1C // REC follows at 0x1D
#define REG_CNF3     0x28 // CNF2 0x29, CNF1 0x2A follow, so one write sets all three
#define REG_CANINTE  0x2B
#define REG_CANINTF  0x2C
#define REG_EFLG     0x2D
#define REG_RXB0CTRL 0x60
#define REG_RXB1CTRL 0x70

#define MODE_MASK   0xE0 // top 3 bits of CANCTRL (request) and CANSTAT (actual)
#define MODE_CONFIG 0x80

// 8 MHz, 500 kbit/s: TQ = 2 x (BRP+1) / 8 MHz = 250 ns. 1 sync + 1 prop + 3 PS1 + 3 PS2 = 8 TQ = 2 us,
// sample point 5/8 = 62.5%. CNF3 bit 7 only routes SOF to the CLKOUT pin. same values as the bring-up
#define CNF3 0x82
#define CNF2 0x90
#define CNF1 0x00
_Static_assert(MCP2515_XTAL_HZ == 8000000 && CAN_BITRATE_BPS == 500000, "CNF1-3 are for 8 MHz, 500 kbit/s");

#define RXM_ANY  0x60 // RXBnCTRL: masks and filters off, take every frame
#define RXB0_BUKT 0x04 // RXB0 full: roll over into RXB1
#define INT_RX   0x03 // RX0IE | RX1IE
#define INTF_ERR 0xA0 // MERRF | ERRIF
#define SIDL_IDE 0x08 // extended frame
#define SIDL_SRR 0x10 // standard remote frame

// one instruction = one CS-low window
static int cmd(const uint8_t *tx, uint8_t *rx, size_t n) {
    spi_cs(true);
    int r = spi_xfer(tx, rx, n);
    spi_cs(false);
    return r;
}

static int read_regs(uint8_t addr, uint8_t *out, size_t n) {
    uint8_t tx[2 + 2] = {INS_READ, addr}, rx[sizeof tx];
    if (n > 2) return -EINVAL;
    int r = cmd(tx, rx, 2 + n);
    memcpy(out, rx + 2, n);
    return r;
}

static int write_regs(uint8_t addr, const uint8_t *v, size_t n) {
    uint8_t tx[2 + 3] = {INS_WRITE, addr};
    if (n > 3) return -EINVAL;
    memcpy(tx + 2, v, n);
    return cmd(tx, NULL, 2 + n);
}

static int write_reg(uint8_t addr, uint8_t v) { return write_regs(addr, &v, 1); }

static int bitmod(uint8_t addr, uint8_t mask, uint8_t v) {
    const uint8_t tx[] = {INS_BITMOD, addr, mask, v};
    return cmd(tx, NULL, sizeof tx);
}

static int set_mode(uint8_t mode) {
    int r = bitmod(REG_CANCTRL, MODE_MASK, mode);
    uint32_t t0 = HAL_GetTick();
    uint8_t stat = 0;
    while (r == 0 && HAL_GetTick() - t0 <= 10) { // the bus must be idle before a mode change completes
        r = read_regs(REG_CANSTAT, &stat, 1);
        if ((stat & MODE_MASK) == mode) return r;
    }
    return r ? r : -EIO;
}

int mcp2515_init(uint8_t mode) {
    int r = spi_init();
    const uint8_t reset = INS_RESET;
    if (r == 0) r = cmd(&reset, NULL, 1);
    if (r) return r;
    HAL_Delay(10); // oscillator restart; the bring-up's library waits 10 ms too

    // after reset it sits in config mode. a dead or unpowered chip reads 0x00 or 0xFF here
    uint8_t stat;
    r = read_regs(REG_CANSTAT, &stat, 1);
    if (r) return r;
    if ((stat & MODE_MASK) != MODE_CONFIG) return -ENODEV;

    const uint8_t cnf[] = {CNF3, CNF2, CNF1};
    if ((r = write_regs(REG_CNF3, cnf, 3))) return r;
    if ((r = write_reg(REG_RXB0CTRL, RXM_ANY | RXB0_BUKT))) return r;
    if ((r = write_reg(REG_RXB1CTRL, RXM_ANY))) return r;
    if ((r = write_reg(REG_CANINTF, 0))) return r;
    if ((r = write_reg(REG_CANINTE, INT_RX))) return r; // INT only for received frames
    return set_mode(mode);
}

int mcp2515_int_init(void) {
    __HAL_RCC_GPIOC_CLK_ENABLE();
    // HAL also turns on SYSCFG and routes port C to EXTI line 7 (EXTICR2)
    GPIO_InitTypeDef g = {.Pin = 1u << CAN_INT_PIN, .Mode = GPIO_MODE_IT_FALLING, .Pull = GPIO_PULLUP};
    HAL_GPIO_Init(CAN_INT_PORT, &g);
    if (mcp2515_int_low()) return -EIO; // init cleared every flag, so low means a stuck line
    __HAL_GPIO_EXTI_CLEAR_IT(1u << CAN_INT_PIN);
    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 3, 0); // below tick 0, I2C 1, PendSV 2; it only sets a flag
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
    return 0;
}

bool mcp2515_int_low(void) { return !(CAN_INT_PORT->IDR & (1u << CAN_INT_PIN)); }

int mcp2515_send(const can_frame_t *f) {
    if (f->dlc > 8 || f->id > 0x7FF) return -EINVAL;
    uint8_t st[2];
    const uint8_t q[2] = {INS_STATUS};
    int r = cmd(q, st, 2);
    if (r) return r;
    int b = 0; // first free TX buffer: STATUS and FEEDBACK can be due in the same pass
    while (b < 3 && (st[1] & (0x04 << (2 * b)))) b++;
    if (b == 3) return -EBUSY;

    // SIDH, SIDL, EID8, EID0, DLC, data
    uint8_t tx[1 + 5 + 8] = {(uint8_t)(INS_LOAD_TX0 + 2 * b), (uint8_t)(f->id >> 3), (uint8_t)((f->id & 7) << 5), 0,
                             0, f->dlc};
    memcpy(tx + 6, f->data, f->dlc);
    if ((r = cmd(tx, NULL, 6 + f->dlc))) return r;
    const uint8_t rts = (uint8_t)(0x80 | (1u << b)); // 0x81, 0x82, 0x84
    return cmd(&rts, NULL, 1);
}

int mcp2515_recv(can_frame_t *f) {
    for (;;) {
        uint8_t st[2];
        const uint8_t q[2] = {INS_STATUS};
        int r = cmd(q, st, 2);
        if (r) return r;
        if (!(st[1] & 0x03)) return -EAGAIN;

        // RXB0 first: with rollover it holds the older frame
        uint8_t tx[1 + 5 + 8] = {st[1] & 0x01 ? INS_READ_RX0 : INS_READ_RX1}, rx[sizeof tx];
        if ((r = cmd(tx, rx, sizeof tx))) return r;
        const uint8_t *b = rx + 1;
        if (b[1] & (SIDL_IDE | SIDL_SRR)) continue; // not ours: extended or remote, already cleared
        f->id = (uint16_t)((b[0] << 3) | (b[1] >> 5));
        f->dlc = b[4] & 0x0F;
        if (f->dlc > 8) f->dlc = 8; // 9..15 are legal on the wire and mean 8
        memcpy(f->data, b + 5, 8);
        return 0;
    }
}

int mcp2515_errors(uint8_t *tec, uint8_t *rec) {
    uint8_t cnt[2], eflg;
    int r = read_regs(REG_TEC, cnt, 2);
    if (r == 0) r = read_regs(REG_EFLG, &eflg, 1);
    // overflow bits latch until cleared; the rest of EFLG follows the counters by itself
    if (r == 0 && (eflg & (MCP2515_EFLG_RX0OVR | MCP2515_EFLG_RX1OVR)))
        r = bitmod(REG_EFLG, MCP2515_EFLG_RX0OVR | MCP2515_EFLG_RX1OVR, 0);
    if (r == 0) r = bitmod(REG_CANINTF, INTF_ERR, 0);
    if (r) return r;
    if (tec) *tec = cnt[0];
    if (rec) *rec = cnt[1];
    return eflg;
}
