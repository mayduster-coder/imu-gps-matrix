#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "main.h"
#include "lwip/udp.h"
#include "lwip/ip_addr.h"
#include "lwip/pbuf.h"
#include "ETH_Telemetry.h"
#include "IMU.h"
extern const uint8_t SMILEY_FACE[8];
extern const uint8_t ARROW_E[8];
extern const uint8_t ARROW_NE[8];
extern const uint8_t ARROW_N[8];
extern const uint8_t ARROW_NW[8];
extern const uint8_t ARROW_W[8];
extern const uint8_t ARROW_SW[8];
extern const uint8_t ARROW_S[8];
extern const uint8_t ARROW_SE[8];
/* ---- provided by freertos.c (FreeRTOS version) ----
 * This file runs inside the lwIP thread, so it must NOT touch I2C / SPI directly
 * (imuTask owns I2C1, matrixTask owns SPI2). It sends requests instead. */
void App_Log(const char *fmt, ...);
void App_EthRxPrint(const uint8_t *data, uint16_t len, const ip_addr_t *addr, u16_t port);
void App_RequestImuZero(void);
void App_ShowPattern(uint8_t pattern_id);

#define ETH_PRINT_TX   1     /* 1 = print every telemetry packet sent (hex) */

static RetryEngine_t retry_engine = {0};
static struct udp_pcb *upcb = NULL;
static uint32_t pkt_counter = 0;

void ETH_Send_Ack(uint32_t seq_num, uint8_t is_ack, const ip_addr_t *addr, u16_t port)
{
    AckPacket_t ack_pkt;
    if (is_ack) // if condition to see if data pass
    {
        ack_pkt.msg_type = MSG_ACK;
    }
    else
    {
        ack_pkt.msg_type = MSG_NACK;
    }    ack_pkt.acked_seq_num = seq_num;

    //Network memory buffer allocation
    struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, sizeof(AckPacket_t), PBUF_RAM);
    if (p != NULL) {
        memcpy(p->payload, &ack_pkt, sizeof(AckPacket_t));
        udp_sendto(upcb, p, addr, port);// function to send packet of data in UPD protocol
        pbuf_free(p);
    }
}
//Scanner and processor of incoming commands from Eth
void ETH_Process_Incoming_Cmd(uint8_t *data, uint16_t len, const ip_addr_t *addr, u16_t port)
{
    // if condition to see if the length of the packet is smaller then 5
    if (len < 5) {
        return;
    }

    uint8_t cmd_id = data[0];
    uint32_t seq_num;
    memcpy(&seq_num, &data[1], sizeof(uint32_t)); // copy data to seq_num

    uint8_t valid = 1;

    switch (cmd_id)
    {
        case 1:
        	App_RequestImuZero();   /* was IMU_CalibrateZero(): I2C from the lwIP thread
        	                           collided with imuTask. imuTask does it now. */
            break;

        case 2:

        	if (len >= 6) {
        	    uint8_t sample_rate = data[5];
        	    if (sample_rate > 100) {
        	    valid = 1;
        	     }
        	 else {
        	     valid = 0;//packet is not in range
        	   }
        	 }
        	else {
        	    valid = 0; //packet is to small
        	  }
        	  break;
        case 3:

              if (len >= 6) {
                        uint8_t pattern_id = data[5];
                        if (pattern_id <= 8) {
                            /* 0 smiley, 1 E, 2 NE, 3 N, 4 NW, 5 W, 6 SW, 7 S, 8 SE.
                             * matrixTask draws it (SPI only from one task) */
                            App_ShowPattern(pattern_id);
                        } else {
                            valid = 0;
                        }
                           }


    else {
           valid = 0;
         }

                    break;

        default:
            valid = 0; // non familiar commend
            break;
    }

    ETH_Send_Ack(seq_num, valid, addr, port);
}
void udp_receive_callback(void *arg, struct udp_pcb *pcb, struct pbuf *p, const ip_addr_t *addr, u16_t port)
{
    if (p != NULL)
    {
        // print the received bytes (hex + bits) on the terminal - done by ethRxTask
        App_EthRxPrint((const uint8_t *)p->payload, p->len, addr, port);

        // Processing incoming callback
        ETH_Process_Incoming_Cmd((uint8_t *)p->payload, p->len, addr, port);

        pbuf_free(p);
    }
}
void ETH_Telemetry_Init(void)
{
    upcb = udp_new();
    if (upcb != NULL)
        {
            // connect to local port
    	    udp_bind(upcb, IP4_ADDR_ANY, 5000);
    	    //  Receive data
            udp_recv(upcb, udp_receive_callback, NULL);
        }
}

void ETH_Telemetry_Send(GPS_GGA_Data *gps_data)
{
    if (upcb == NULL) return;

    ip_addr_t destAddr;
    //Configure and set up an IPv4 address.
    IP4_ADDR(&destAddr, DEST_IP_ADDR0, DEST_IP_ADDR1, DEST_IP_ADDR2, DEST_IP_ADDR3);

    TelemetryPacket pkt = {0};
    pkt.packet_id = ++pkt_counter;

    // extract GPS data
    if (gps_data != NULL)
    {
        pkt.latitude    = gps_data->latitude;
        pkt.longitude   = gps_data->longitude;
        pkt.altitude    = gps_data->altitude;
        pkt.fix_quality = gps_data->fix_quality;
    }

    // extract IMU data
    pkt.accel_x = imu_data.accel_x;
    pkt.accel_y = imu_data.accel_y;
    pkt.accel_z = imu_data.accel_z;
    pkt.gyro_x  = imu_data.gyro_x;
    pkt.gyro_y  = imu_data.gyro_y;
    pkt.gyro_z  = imu_data.gyro_z;


    struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, sizeof(TelemetryPacket), PBUF_RAM);
    if (p != NULL)
    {
        memcpy(p->payload, &pkt, sizeof(TelemetryPacket));
        udp_sendto(upcb, p, &destAddr, DEST_PORT);// send in UPD
        pbuf_free(p);
    }

#if ETH_PRINT_TX
    // print bytes to terminal - built as ONE line, printed with the mutex (no mixing)
    uint8_t *raw_bytes = (uint8_t *)&pkt;
    char line[220];
    int pos = snprintf(line, sizeof line, "[ETH TX #%lu, %u B] ",
                       (unsigned long)pkt_counter, (unsigned int)sizeof(TelemetryPacket));
    for (size_t i = 0; i < sizeof(TelemetryPacket) && pos < (int)sizeof(line) - 4; i++)
    {
        pos += snprintf(&line[pos], sizeof line - (size_t)pos, "%02X ", raw_bytes[i]);
    }
    App_Log("%s\r\n", line);
#endif
}
//Reliable transmission mechanism over UDP
void ETH_Send_With_Retry(uint8_t *data, uint16_t len, uint32_t seq_num, const ip_addr_t *dest_ip, uint16_t dest_port)
{
	if (len > sizeof(retry_engine.tx_buffer)) {
	    return;
	}
    // Save data in retry_engine
    memcpy(retry_engine.tx_buffer, data, len);
    retry_engine.tx_len = len;
    retry_engine.seq_num = seq_num;
    retry_engine.dest_ip = *dest_ip;
    retry_engine.dest_port = dest_port;
    retry_engine.retry_count = 0;
    retry_engine.is_waiting_ack = 1;
    retry_engine.last_send_time = HAL_GetTick();

    struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, len, PBUF_RAM);
    if (p != NULL) {
        memcpy(p->payload, data, len);
        udp_sendto(upcb, p, dest_ip, dest_port);
        pbuf_free(p);
    }
}

void ETH_Process_ACK(uint32_t acked_seq)
{
    if (retry_engine.is_waiting_ack && (retry_engine.seq_num == acked_seq))// the call succeed
    {
        retry_engine.is_waiting_ack = 0; // clean ACK engine
    }
}


void ETH_Retry_Task(void) // A function that performs retransmissions
{
    if (!retry_engine.is_waiting_ack) return;

    // check every 500ms
    if (HAL_GetTick() - retry_engine.last_send_time >= TIMEOUT_MS)
    {
        if (retry_engine.retry_count < MAX_RETRIES)
        {
            retry_engine.retry_count++;
            retry_engine.last_send_time = HAL_GetTick();

            // Resends packet
            struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, retry_engine.tx_len, PBUF_RAM);// declare p struct
            if (p != NULL) {
                memcpy(p->payload, retry_engine.tx_buffer, retry_engine.tx_len);// copy retry_engine to p
                udp_sendto(upcb, p, &retry_engine.dest_ip, retry_engine.dest_port); // send data to PC
                pbuf_free(p);
            }
        }
        else
        {
            retry_engine.is_waiting_ack = 0;
        }
    }
}
