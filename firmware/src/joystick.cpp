// 8BitDo Pro 2 over Bluetooth Classic HID → manual cart jog + buttons.
// Active only when currentState == JOYSTICK; left-stick X drives PWM via
// update_motor_directly(), bypassing the LQR motor model. Button state and
// rumble output are exposed via joystick.h for use by main.cpp. Requires
// the Pico W build (BT-enabled arduino-pico core).

#include <Arduino.h>
#include "hardware.h"
#include "states.h"
#include "joystick.h"

extern "C" {
#include "btstack.h"
#include "pico/cyw43_arch.h"
#include "classic/hid_host.h"
}

static constexpr uint8_t  AXIS_DEADZONE       = 20;
static constexpr uint8_t  AXIS_CENTRE         = 127;
static constexpr int      MAX_JOY_PWM         = 255;
static constexpr uint32_t INACTIVE_TIMEOUT_MS = 500;
static constexpr uint8_t  INQUIRY_DURATION    = 3;

namespace {

// Per the 8BitDo Pro 2 mode D layout (axes empirically verified, button
// bytes still needing verification): r[1]=LX, r[2]=LY, r[3]=RX, r[4]=RY,
// r[5]=face/shoulder buttons, r[6]=Select/Start/Home/L3/R3, r[7]=hat/dpad.
// The bit table below is a best guess — the [JOY-BTN] debug print emits
// any change in r[5..7] so positions can be confirmed on the bench.
struct ButtonInfo { uint8_t byte_idx; uint8_t bit_mask; };
constexpr ButtonInfo BUTTON_TABLE[JOY_BTN_COUNT] = {
    /* JOY_BTN_A      */ {5, 0x01},
    /* JOY_BTN_B      */ {5, 0x02},
    /* JOY_BTN_X      */ {5, 0x08},
    /* JOY_BTN_Y      */ {5, 0x10},
    /* JOY_BTN_L1     */ {5, 0x40},
    /* JOY_BTN_R1     */ {5, 0x80},
    /* JOY_BTN_SELECT */ {6, 0x04},
    /* JOY_BTN_START  */ {6, 0x08},
    /* JOY_BTN_HOME   */ {6, 0x10},
};

struct PadState {
    uint8_t  lx = AXIS_CENTRE;
    uint8_t  btn_bytes[3]      = {0, 0, 0}; // raw r[5], r[6], r[7]
    uint8_t  btn_bytes_prev[3] = {0, 0, 0}; // for [JOY-BTN] change detection
    uint16_t pressed_edge      = 0;          // bit i set on rising edge for button i
    uint32_t last_report_ms    = 0;
    bool     connected         = false;
};
PadState g_pad;

uint16_t   g_hid_cid           = 0;
bool       g_connecting        = false;
bool       g_gamepad_seen      = false;
bd_addr_t  g_last_gamepad_addr = {0};

uint8_t                                 g_hid_descriptor[1000];
btstack_packet_callback_registration_t  g_hci_event_registration;

int lx_to_pwm() {
    int d = AXIS_CENTRE - (int)g_pad.lx;
    if (d > -AXIS_DEADZONE && d < AXIS_DEADZONE) return 0;
    if (d > 0) d -= AXIS_DEADZONE;
    else       d += AXIS_DEADZONE;
    int pwm = (int)((float)d / (127.0f - (float)AXIS_DEADZONE) * (float)MAX_JOY_PWM);
    if (pwm >  MAX_JOY_PWM) pwm =  MAX_JOY_PWM;
    if (pwm < -MAX_JOY_PWM) pwm = -MAX_JOY_PWM;
    return pwm;
}

bool button_held_internal(uint8_t btn) {
    if (btn >= JOY_BTN_COUNT) return false;
    const ButtonInfo& b = BUTTON_TABLE[btn];
    if (b.byte_idx < 5 || b.byte_idx > 7) return false;
    return (g_pad.btn_bytes[b.byte_idx - 5] & b.bit_mask) != 0;
}

void handle_report(const uint8_t* r, uint16_t n) {
    if (n < 2) return;
    g_pad.lx = r[1];

    // Buttons live at r[5..7]; gate so we don't read garbage on short reports.
    if (n >= 8) {
        for (int i = 0; i < 3; i++) {
            g_pad.btn_bytes_prev[i] = g_pad.btn_bytes[i];
            g_pad.btn_bytes[i]      = r[5 + i];
        }
        // Compute press edges (0 → 1 on each button bit) into pressed_edge.
        // OR-accumulates; consumers clear by calling joystick_consume_press.
        // Critical section: pressed_edge is read-modify-written from the
        // main loop and ORed here from a BTstack async callback.
        uint16_t edges_now = 0;
        for (uint8_t b = 0; b < JOY_BTN_COUNT; b++) {
            const ButtonInfo& info = BUTTON_TABLE[b];
            if (info.byte_idx < 5 || info.byte_idx > 7) continue;
            uint8_t cur  = g_pad.btn_bytes     [info.byte_idx - 5] & info.bit_mask;
            uint8_t prev = g_pad.btn_bytes_prev[info.byte_idx - 5] & info.bit_mask;
            if (cur && !prev) edges_now |= (uint16_t)(1u << b);
        }
        if (edges_now) {
            noInterrupts();
            g_pad.pressed_edge |= edges_now;
            interrupts();
        }
        // Debug print on any change to r[5..7] so bit positions can be
        // verified on the bench. Drops itself once button mapping is solid.
        bool changed = false;
        for (int i = 0; i < 3; i++) {
            if (g_pad.btn_bytes[i] != g_pad.btn_bytes_prev[i]) { changed = true; break; }
        }
        if (changed) {
            Serial.print("[JOY-BTN] r5=0x"); Serial.print(g_pad.btn_bytes[0], HEX);
            Serial.print(" r6=0x");          Serial.print(g_pad.btn_bytes[1], HEX);
            Serial.print(" r7=0x");          Serial.println(g_pad.btn_bytes[2], HEX);
        }
    }

    g_pad.last_report_ms = millis();
}

void packet_handler(uint8_t pkt_type, uint16_t /*ch*/, uint8_t* pkt, uint16_t /*sz*/) {
    if (pkt_type != HCI_EVENT_PACKET) return;
    uint8_t event = hci_event_packet_get_type(pkt);

    switch (event) {
        case BTSTACK_EVENT_STATE:
            if (btstack_event_state_get_state(pkt) == HCI_STATE_WORKING) {
                Serial.println("[JOY] BT ready, scanning for controller");
                gap_inquiry_start(INQUIRY_DURATION);
            }
            break;

        case GAP_EVENT_INQUIRY_RESULT: {
            bd_addr_t addr;
            gap_event_inquiry_result_get_bd_addr(pkt, addr);
            uint32_t cod   = gap_event_inquiry_result_get_class_of_device(pkt);
            uint8_t  major = (cod >> 8) & 0x1F;
            if (major == 0x05) {
                g_gamepad_seen = true;
                memcpy(g_last_gamepad_addr, addr, 6);
                Serial.print("[JOY] gamepad seen: ");
                Serial.println(bd_addr_to_str(addr));
            }
            break;
        }

        case GAP_EVENT_INQUIRY_COMPLETE:
            if (g_hid_cid != 0 || g_connecting) break;
            if (g_gamepad_seen) {
                g_connecting = true;
                uint8_t st = hid_host_connect(g_last_gamepad_addr,
                                              HID_PROTOCOL_MODE_REPORT,
                                              &g_hid_cid);
                if (st != ERROR_CODE_SUCCESS) {
                    Serial.print("[JOY] hid_host_connect failed 0x");
                    Serial.println(st, HEX);
                    g_connecting   = false;
                    g_gamepad_seen = false;
                    gap_inquiry_start(INQUIRY_DURATION);
                }
            } else {
                gap_inquiry_start(INQUIRY_DURATION);
            }
            break;

        case HCI_EVENT_PIN_CODE_REQUEST: {
            bd_addr_t addr;
            hci_event_pin_code_request_get_bd_addr(pkt, addr);
            gap_pin_code_response(addr, "0000");
            break;
        }

        case HCI_EVENT_USER_CONFIRMATION_REQUEST: {
            bd_addr_t addr;
            hci_event_user_confirmation_request_get_bd_addr(pkt, addr);
            gap_ssp_confirmation_response(addr);
            break;
        }

        // Force re-pairing every boot — simpler than persisting link keys.
        case HCI_EVENT_LINK_KEY_REQUEST: {
            bd_addr_t addr;
            hci_event_link_key_request_get_bd_addr(pkt, addr);
            gap_drop_link_key_for_bd_addr(addr);
            hci_send_cmd(&hci_link_key_request_negative_reply, addr);
            break;
        }

        case HCI_EVENT_HID_META: {
            uint8_t sub = hci_event_hid_meta_get_subevent_code(pkt);
            switch (sub) {
                case HID_SUBEVENT_INCOMING_CONNECTION: {
                    uint16_t cid = hid_subevent_incoming_connection_get_hid_cid(pkt);
                    g_connecting = true;
                    hid_host_accept_connection(cid, HID_PROTOCOL_MODE_REPORT);
                    break;
                }
                case HID_SUBEVENT_CONNECTION_OPENED: {
                    uint8_t st = hid_subevent_connection_opened_get_status(pkt);
                    g_connecting = false;
                    if (st != ERROR_CODE_SUCCESS) {
                        Serial.print("[JOY] connection FAILED 0x");
                        Serial.println(st, HEX);
                        g_hid_cid      = 0;
                        g_gamepad_seen = false;
                        gap_inquiry_start(INQUIRY_DURATION);
                        break;
                    }
                    g_hid_cid            = hid_subevent_connection_opened_get_hid_cid(pkt);
                    g_pad.connected      = true;
                    g_pad.lx             = AXIS_CENTRE;
                    memset(g_pad.btn_bytes,      0, sizeof(g_pad.btn_bytes));
                    memset(g_pad.btn_bytes_prev, 0, sizeof(g_pad.btn_bytes_prev));
                    g_pad.pressed_edge   = 0;
                    g_pad.last_report_ms = millis();
                    Serial.println("[JOY] controller CONNECTED");
                    break;
                }
                case HID_SUBEVENT_CONNECTION_CLOSED:
                    Serial.println("[JOY] controller DISCONNECTED, rescanning");
                    g_hid_cid          = 0;
                    g_connecting       = false;
                    g_gamepad_seen     = false;
                    g_pad.connected    = false;
                    g_pad.lx           = AXIS_CENTRE;
                    memset(g_pad.btn_bytes,      0, sizeof(g_pad.btn_bytes));
                    memset(g_pad.btn_bytes_prev, 0, sizeof(g_pad.btn_bytes_prev));
                    g_pad.pressed_edge = 0;
                    gap_inquiry_start(INQUIRY_DURATION);
                    break;
                case HID_SUBEVENT_REPORT: {
                    const uint8_t* r = hid_subevent_report_get_report(pkt);
                    uint16_t       n = hid_subevent_report_get_report_len(pkt);
                    if (n > 1 && r[0] == 0xA1) handle_report(r + 1, n - 1);
                    else                       handle_report(r,     n);
                    break;
                }
                default: break;
            }
            break;
        }

        default: break;
    }
}

}  // anonymous namespace

// ---- Rumble engine ----------------------------------------------------------
// Internal state for the rumble engine. Pulses (one-shot, override) and a
// continuous baseline level are tracked separately; joystick_rumble_tick()
// resolves them into a single intensity and pushes a SET_REPORT to the
// controller, rate-limited and refreshed periodically (some controllers
// stop rumbling without periodic refresh reports).
namespace {
uint8_t  g_rumble_continuous   = 0;
uint32_t g_rumble_pulse_end_ms = 0;
uint8_t  g_rumble_pulse_strong = 0;
uint8_t  g_last_rumble_sent    = 0;
uint32_t g_last_rumble_send_ms = 0;

// FORMAT IS UNVERIFIED for 8BitDo Pro 2 mode D — this is a best-guess
// based on common gamepad layouts (8-byte output report, big motor at
// index 2). If rumble doesn't trigger, alternatives to try:
//   - report_id 0x01 instead of 0x05
//   - swap indices 2/3 (some controllers use [weak, strong])
//   - drop or change report_id (some need 0)
void send_rumble_raw(uint8_t strong) {
    if (!g_pad.connected || g_hid_cid == 0) return;
    uint8_t report[8] = {0x00, 0x00, strong, 0x00, 0x00, 0x00, 0x00, 0x00};
    hid_host_send_set_report(g_hid_cid, HID_REPORT_TYPE_OUTPUT, 0x05,
                             report, sizeof(report));
}
}  // anonymous namespace

void joystick_rumble_pulse(uint8_t strong, uint16_t duration_ms) {
    g_rumble_pulse_strong = strong;
    g_rumble_pulse_end_ms = millis() + duration_ms;
}

void joystick_rumble_continuous(uint8_t strong) {
    g_rumble_continuous = strong;
}

void joystick_rumble_tick() {
    if (!g_pad.connected) {
        g_last_rumble_sent = 0;
        return;
    }
    uint32_t now = millis();
    uint8_t target = (now < g_rumble_pulse_end_ms) ? g_rumble_pulse_strong
                                                   : g_rumble_continuous;
    bool changed     = (target != g_last_rumble_sent);
    bool refresh_due = (target > 0) && (now - g_last_rumble_send_ms > 200);
    if (changed || refresh_due) {
        send_rumble_raw(target);
        g_last_rumble_sent    = target;
        g_last_rumble_send_ms = now;
    }
}

// ---- Setup / connection -----------------------------------------------------

void joystick_setup() {
    Serial.println("[JOY] joystick_setup()");

    l2cap_init();
    sdp_init();

    gap_set_local_name("Pico W Pendulum 00:00:00:00:00:00");
    gap_set_class_of_device(0x2540);
    gap_set_allow_role_switch(true);
    gap_discoverable_control(1);
    gap_connectable_control(1);
    gap_ssp_set_io_capability(SSP_IO_CAPABILITY_NO_INPUT_NO_OUTPUT);
    gap_ssp_set_auto_accept(1);

    hid_host_init(g_hid_descriptor, sizeof(g_hid_descriptor));
    hid_host_register_packet_handler(&packet_handler);

    g_hci_event_registration.callback = &packet_handler;
    hci_add_event_handler(&g_hci_event_registration);

    hci_power_control(HCI_POWER_ON);
    // Don't call btstack_run_loop_execute() — Philhower's arduino-pico core
    // services BTstack from its own async_context.
}

bool joystick_drive_motor_direct() {
    if (!g_pad.connected) return false;
    if (millis() - g_pad.last_report_ms > INACTIVE_TIMEOUT_MS) {
        coast_motor();
        return false;
    }
    int pwm = lx_to_pwm();
    if (pwm == 0) {
        coast_motor();
    } else {
        ENA = pwm;
        update_motor_directly();
    }
    return true;
}

void joystick_tick(float /*xdot*/) {
    if (!g_pad.connected) {
        if (currentState == JOYSTICK) {
            currentState = IDLE;
            event = "joystick_disconnect";
            coast_motor();
        }
        return;
    }
    if (currentState != JOYSTICK) return;
    joystick_drive_motor_direct();
}

bool joystick_connected() {
    return g_pad.connected;
}

bool joystick_button_held(uint8_t btn) {
    return button_held_internal(btn);
}

bool joystick_consume_press(uint8_t btn) {
    if (btn >= JOY_BTN_COUNT) return false;
    uint16_t mask = (uint16_t)(1u << btn);
    bool was_set;
    noInterrupts();
    was_set = (g_pad.pressed_edge & mask) != 0;
    if (was_set) g_pad.pressed_edge &= ~mask;
    interrupts();
    return was_set;
}

float joystick_lx_normalised() {
    int d = AXIS_CENTRE - (int)g_pad.lx;
    if (d > -AXIS_DEADZONE && d < AXIS_DEADZONE) return 0.0f;
    if (d > 0) d -= AXIS_DEADZONE;
    else       d += AXIS_DEADZONE;
    float norm = (float)d / (127.0f - (float)AXIS_DEADZONE);
    if (norm >  1.0f) norm =  1.0f;
    if (norm < -1.0f) norm = -1.0f;
    return norm;
}

