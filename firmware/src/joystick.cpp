// 8BitDo Pro 2 over Bluetooth Classic HID → manual cart jog.
// Active only when currentState == JOYSTICK; left-stick X drives PWM via
// update_motor_directly(), bypassing the LQR motor model.

#include <Arduino.h>
#include "hardware.h"
#include "states.h"

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

// Button byte index and bitmasks for 8BitDo in D-mode.
// If buttons don't respond correctly, uncomment JOY_BTN_DEBUG below and
// check the serial output while pressing each button to find the right masks.
// #define JOY_BTN_DEBUG
static constexpr uint8_t BTN_BYTE   = 5;
static constexpr uint8_t BTN_X_MASK = 0x08;  // X  → IDLE
static constexpr uint8_t BTN_Y_MASK = 0x04;  // Y  → JOYSTICK swing-up
static constexpr uint8_t BTN_A_MASK = 0x01;  // A  → AUTO_SWINGUP

namespace {

struct PadState {
    uint8_t  lx           = AXIS_CENTRE;
    uint8_t  buttons      = 0;
    uint8_t  prev_buttons = 0;
    uint32_t last_report_ms = 0;
    bool     connected = false;
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

// 8BitDo Pro 2 mode D: after stripping the 0xA1 HID-input prefix, axis
// bytes are LX=r[1], LY=r[2], RX=r[3], RY=r[4]. Only LX is used.
void handle_report(const uint8_t* r, uint16_t n) {
    if (n < 2) return;
    g_pad.lx = r[1];
    g_pad.last_report_ms = millis();

    if (n > BTN_BYTE) {
        g_pad.prev_buttons = g_pad.buttons;
        g_pad.buttons      = r[BTN_BYTE];
        uint8_t pressed    = g_pad.buttons & ~g_pad.prev_buttons;

        if (pressed & BTN_X_MASK) {
            currentState = IDLE;
            event = "button_idle";
            coast_motor();
        } else if (pressed & BTN_Y_MASK) {
            currentState = JOYSTICK;
            event = "button_joystick";
        } else if (pressed & BTN_A_MASK) {
            currentState = AUTO_SWINGUP;
            event = "button_auto_swingup";
        }
    }

#ifdef JOY_BTN_DEBUG
    Serial.print("[BTN] n="); Serial.print(n);
    for (uint16_t i = 0; i < n && i < 10; i++) {
        Serial.print(" ["); Serial.print(i); Serial.print("]=0x");
        Serial.print(r[i], HEX);
    }
    Serial.println();
#endif
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
                    g_pad.last_report_ms = millis();
                    Serial.println("[JOY] controller CONNECTED");
                    break;
                }
                case HID_SUBEVENT_CONNECTION_CLOSED:
                    Serial.println("[JOY] controller DISCONNECTED, rescanning");
                    g_hid_cid       = 0;
                    g_connecting    = false;
                    g_gamepad_seen  = false;
                    g_pad.connected = false;
                    g_pad.lx        = AXIS_CENTRE;
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

    if (millis() - g_pad.last_report_ms > INACTIVE_TIMEOUT_MS) {
        coast_motor();
        return;
    }

    int pwm = lx_to_pwm();
    if (pwm == 0) {
        coast_motor();
        return;
    }
    ENA = pwm;
    update_motor_directly();
}

bool joystick_connected() {
    return g_pad.connected;
}
