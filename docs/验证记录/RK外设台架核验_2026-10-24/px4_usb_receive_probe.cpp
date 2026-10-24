// Temporary receive-only diagnostic for an unbound PX4 CDC-ACM USB device.
// Default: receive only. Optional --gcs-heartbeat sends only GCS heartbeats.
// Optional --gps-query adds two REQUEST_MESSAGE commands, for GPS data only.
// No setpoints, parameters or mode/arming messages are sent.
// No driver is detached or installed.
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <map>
#include <mavlink/v2.0/common/mavlink.h>

// Stable libusb-1.0 C ABI; the board has the runtime library but no dev headers.
extern "C" {
struct libusb_context;
struct libusb_device_handle;
int libusb_init(libusb_context **);
void libusb_exit(libusb_context *);
libusb_device_handle *libusb_open_device_with_vid_pid(libusb_context *, uint16_t, uint16_t);
int libusb_kernel_driver_active(libusb_device_handle *, int);
int libusb_claim_interface(libusb_device_handle *, int);
int libusb_release_interface(libusb_device_handle *, int);
int libusb_control_transfer(libusb_device_handle *, uint8_t, uint8_t, uint16_t, uint16_t, unsigned char *, uint16_t, unsigned int);
int libusb_bulk_transfer(libusb_device_handle *, unsigned char, unsigned char *, int, int *, unsigned int);
void libusb_close(libusb_device_handle *);
}

int main(int argc, char **argv) {
    bool gps_query = argc == 2 && std::strcmp(argv[1], "--gps-query") == 0;
    bool send_heartbeat = gps_query || (argc == 2 && std::strcmp(argv[1], "--gcs-heartbeat") == 0);
    setvbuf(stdout, nullptr, _IOLBF, 0);
    libusb_context *ctx = nullptr;
    if (libusb_init(&ctx) != 0) return 2;
    auto *h = libusb_open_device_with_vid_pid(ctx, 0x3185, 0x0038);
    if (!h) { std::puts("USB_OPEN_FAILED"); libusb_exit(ctx); return 3; }
    bool claimed[2] = {false, false};
    for (int i = 0; i < 2; ++i) {
        int active = libusb_kernel_driver_active(h, i);
        std::printf("interface=%d kernel_driver_active=%d\n", i, active);
        if (active != 0) { libusb_close(h); libusb_exit(ctx); return 4; }
    }
    for (int i = 0; i < 2; ++i) {
        int r = libusb_claim_interface(h, i);
        std::printf("claim_interface=%d result=%d\n", i, r);
        if (r != 0) {
            for (int j = 0; j < i; ++j) libusb_release_interface(h, j);
            libusb_close(h); libusb_exit(ctx); return 5;
        }
        claimed[i] = true;
    }
    // CDC line coding 115200, 8N1 and DTR/RTS. These are transport controls.
    unsigned char coding[7] = {0x00, 0xc2, 0x01, 0x00, 0, 0, 8};
    int line = libusb_control_transfer(h, 0x21, 0x20, 0, 0, coding, 7, 1000);
    int dtr = libusb_control_transfer(h, 0x21, 0x22, 3, 0, nullptr, 0, 1000);
    std::printf("cdc_line_coding_result=%d cdc_dtr_result=%d\n", line, dtr);
    std::map<unsigned int, unsigned int> counts;
    unsigned long bytes = 0;
    unsigned long heartbeat_bytes = 0;
    unsigned long gps_query_bytes = 0;
    unsigned int errors = 0;
    mavlink_message_t msg{};
    mavlink_status_t status{};
    auto start = std::chrono::steady_clock::now();
    double next_heartbeat = 0;
    unsigned int requests = 0;
    while (std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count() < 22) {
        double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();
        if (send_heartbeat && elapsed >= next_heartbeat) {
            mavlink_message_t heartbeat;
            mavlink_msg_heartbeat_pack(255, 190, &heartbeat, MAV_TYPE_GCS, MAV_AUTOPILOT_INVALID, 0, 0, MAV_STATE_ACTIVE);
            unsigned char packet[MAVLINK_MAX_PACKET_LEN];
            int length = mavlink_msg_to_send_buffer(packet, &heartbeat);
            int written = 0;
            int sent = libusb_bulk_transfer(h, 0x03, packet, length, &written, 500);
            if (sent != 0) { std::printf("heartbeat_write_error=%d\n",sent); ++errors; break; }
            heartbeat_bytes += written;
            next_heartbeat = elapsed + 1;
        }
        if (gps_query && requests < 2 && elapsed >= 2 + requests * 4) {
            unsigned int requested_id = requests == 0 ? MAVLINK_MSG_ID_GPS_RAW_INT : MAVLINK_MSG_ID_GPS2_RAW;
            mavlink_message_t query;
            mavlink_msg_command_long_pack(255,190,&query,1,1,MAV_CMD_REQUEST_MESSAGE,0,requested_id,0,0,0,0,0,0);
            unsigned char packet[MAVLINK_MAX_PACKET_LEN];
            int length=mavlink_msg_to_send_buffer(packet,&query), written=0;
            int sent=libusb_bulk_transfer(h,0x03,packet,length,&written,500);
            std::printf("GPS_MESSAGE_REQUEST id=%u result=%d bytes=%d\n",requested_id,sent,written);
            gps_query_bytes += written;
            ++requests;
        }
        unsigned char data[4096];
        int transferred = 0;
        int r = libusb_bulk_transfer(h, 0x82, data, sizeof(data), &transferred, 250);
        if (r != 0 && r != -7) { ++errors; std::printf("usb_read_error=%d\n", r); break; }
        bytes += transferred;
        for (int i = 0; i < transferred; ++i) {
            if (!mavlink_parse_char(MAVLINK_COMM_0, data[i], &msg, &status)) continue;
            unsigned int n = ++counts[msg.msgid];
            if (n > 1 && msg.msgid != MAVLINK_MSG_ID_STATUSTEXT) continue;
            if (n > 5) continue;
            std::printf("MESSAGE id=%u sys=%u comp=%u ", msg.msgid, msg.sysid, msg.compid);
            switch (msg.msgid) {
            case MAVLINK_MSG_ID_HEARTBEAT: {
                mavlink_heartbeat_t p; mavlink_msg_heartbeat_decode(&msg, &p);
                std::printf("HEARTBEAT type=%u autopilot=%u base_mode=%u armed=%u custom_mode=%u system_status=%u", p.type, p.autopilot, p.base_mode, !!(p.base_mode & MAV_MODE_FLAG_SAFETY_ARMED), p.custom_mode, p.system_status); break;
            }
            case MAVLINK_MSG_ID_GPS_RAW_INT: {
                mavlink_gps_raw_int_t p; mavlink_msg_gps_raw_int_decode(&msg, &p);
                std::printf("GPS_RAW_INT fix_type=%u satellites=%u lat=%.7f lon=%.7f alt_m=%.3f eph=%u epv=%u", p.fix_type, p.satellites_visible, p.lat/1e7, p.lon/1e7, p.alt/1000., p.eph, p.epv); break;
            }
            case MAVLINK_MSG_ID_GPS2_RAW: {
                mavlink_gps2_raw_t p; mavlink_msg_gps2_raw_decode(&msg,&p);
                std::printf("GPS2_RAW fix_type=%u satellites=%u lat=%.7f lon=%.7f",p.fix_type,p.satellites_visible,p.lat/1e7,p.lon/1e7); break;
            }
            case MAVLINK_MSG_ID_COMMAND_ACK: {
                mavlink_command_ack_t p; mavlink_msg_command_ack_decode(&msg,&p);
                std::printf("COMMAND_ACK command=%u result=%u",p.command,p.result); break;
            }
            case MAVLINK_MSG_ID_AUTOPILOT_VERSION: {
                mavlink_autopilot_version_t p; mavlink_msg_autopilot_version_decode(&msg,&p);
                std::printf("AUTOPILOT_VERSION flight_sw_version=0x%08x board_version=%u vendor_id=%u product_id=%u",p.flight_sw_version,p.board_version,p.vendor_id,p.product_id); break;
            }
            case MAVLINK_MSG_ID_ATTITUDE: {
                mavlink_attitude_t p; mavlink_msg_attitude_decode(&msg, &p);
                std::printf("ATTITUDE roll=%.5f pitch=%.5f yaw=%.5f", p.roll, p.pitch, p.yaw); break;
            }
            case MAVLINK_MSG_ID_HIGHRES_IMU: {
                mavlink_highres_imu_t p; mavlink_msg_highres_imu_decode(&msg, &p);
                std::printf("HIGHRES_IMU accel=(%.4f,%.4f,%.4f) gyro=(%.4f,%.4f,%.4f)", p.xacc,p.yacc,p.zacc,p.xgyro,p.ygyro,p.zgyro); break;
            }
            case MAVLINK_MSG_ID_SYS_STATUS: {
                mavlink_sys_status_t p; mavlink_msg_sys_status_decode(&msg, &p);
                std::printf("SYS_STATUS present=%u enabled=%u health=%u voltage_battery_mV=%u", p.onboard_control_sensors_present,p.onboard_control_sensors_enabled,p.onboard_control_sensors_health,p.voltage_battery); break;
            }
            case MAVLINK_MSG_ID_STATUSTEXT: {
                mavlink_statustext_t p; mavlink_msg_statustext_decode(&msg, &p);
                char text[51]{}; std::memcpy(text,p.text,50);
                std::printf("STATUSTEXT severity=%u text=%s",p.severity,text); break;
            }
            default: std::printf("payload_length=%u",msg.len);
            }
            std::puts("");
        }
    }
    std::printf("SUMMARY seconds=%.3f bytes=%lu gcs_heartbeat_bytes=%lu gps_query_bytes=%lu usb_errors=%u\n",std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count(),bytes,heartbeat_bytes,gps_query_bytes,errors);
    for (auto p : counts) std::printf("COUNT id=%u count=%u\n",p.first,p.second);
    std::printf("parser_received=%u parser_drop=%u parser_errors=%u\n",status.packet_rx_success_count,status.packet_rx_drop_count,status.parse_error);
    int reset = libusb_control_transfer(h,0x21,0x22,0,0,nullptr,0,1000);
    std::printf("cdc_dtr_release_result=%d\n",reset);
    for (int i = 1; i >= 0; --i) if (claimed[i]) libusb_release_interface(h,i);
    libusb_close(h); libusb_exit(ctx);
    return counts[MAVLINK_MSG_ID_HEARTBEAT] ? 0 : 6;
}
