#ifndef ETH_TELEMETRY_H
#define ETH_TELEMETRY_H

#include <stdint.h>
#include "GPS.h"
#include "IMU.h"
#include "lwip/ip_addr.h"
#include "lwip/udp.h"
#include "lwip/pbuf.h"
#define DEST_IP_ADDR0   192
#define DEST_IP_ADDR1   168
#define DEST_IP_ADDR2   1
#define DEST_IP_ADDR3   10   // CP address
#define DEST_PORT       5005
#define MSG_ACK   0x06
#define MSG_NACK  0x15
#define MAX_RETRIES       3
#define TIMEOUT_MS        500
#pragma pack(push, 1)

typedef struct {
    uint8_t  msg_type;       // MSG_ACK or MSG_NACK
    uint32_t acked_seq_num;  // number of packets
} AckPacket_t;

#pragma pack(pop)
void udp_receive_callback(void *arg, struct udp_pcb *pcb, struct pbuf *p, const ip_addr_t *addr, uint16_t port);
void ETH_Send_Ack(uint32_t seq_num, uint8_t is_ack, const ip_addr_t *addr, u16_t port);
void ETH_Process_Incoming_Cmd(uint8_t *data, uint16_t len, const ip_addr_t *addr, u16_t port);
typedef struct {
    uint32_t packet_id;
    float    latitude;
    float    longitude;
    float    altitude;
    uint8_t  fix_quality;
    float    accel_x;
    float    accel_y;
    float    accel_z;
    float    gyro_x;
    float    gyro_y;
    float    gyro_z;
} TelemetryPacket;

typedef struct {
    uint8_t  tx_buffer[128];
    uint16_t tx_len;
    uint32_t seq_num;
    uint32_t last_send_time;
    uint8_t  retry_count;
    uint8_t  is_waiting_ack;
    ip_addr_t dest_ip;
    uint16_t  dest_port;
} RetryEngine_t;

void ETH_Send_With_Retry(uint8_t *data, uint16_t len, uint32_t seq_num, const ip_addr_t *dest_ip, uint16_t dest_port);
void ETH_Process_ACK(uint32_t acked_seq);
void ETH_Retry_Task(void);
void ETH_Telemetry_Init(void);
void ETH_Telemetry_Send(GPS_GGA_Data *gps_data);

#endif // ETH_TELEMETRY_H
