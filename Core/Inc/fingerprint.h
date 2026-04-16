#ifndef __FINGERPRINT_H
#define __FINGERPRINT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

#ifdef USE_FREERTOS
  #include "FreeRTOS.h"
  #include "task.h"
  #define FP_DELAY(ms)  vTaskDelay(pdMS_TO_TICKS(ms))
#else
  #define FP_DELAY(ms)  HAL_Delay(ms)
#endif

/* UART dùng cho cảm biến */
extern UART_HandleTypeDef huart3;
#define UART_HANDLER huart3


#define DEFAULTTIMEOUT   1500U
#define MODELTIMEOUT     3000U

/* Packet constants */
#define FINGERPRINT_STARTCODE       0xEF01
#define FINGERPRINT_COMMANDPACKET   0x01
#define FINGERPRINT_DATAPACKET      0x02
#define FINGERPRINT_ACKPACKET       0x07
#define FINGERPRINT_ENDDATAPACKET   0x08
#define FINGERPRINT_TEMPLATEINDEX   0x1FU //themvaosau
/* Capacity */
#define FINGER_CAPACITY             127U

/* Commands */
#define FINGERPRINT_GETIMAGE        0x01
#define FINGERPRINT_IMAGE2TZ        0x02
#define FINGERPRINT_SEARCH          0x04
#define FINGERPRINT_REGMODEL        0x05
#define FINGERPRINT_STORE           0x06
#define FINGERPRINT_DELETE          0x0C
#define FINGERPRINT_EMPTY           0x0D
#define FINGERPRINT_READSYSPARAM    0x0F
#define FINGERPRINT_TEMPLATECOUNT   0x1D
#define FINGERPRINT_LEDON           0x50
#define FINGERPRINT_LEDOFF          0x51

/* ACK codes */
#define FINGERPRINT_OK               0x00
#define FINGERPRINT_PACKETRECIEVEERR 0x01
#define FINGERPRINT_NOFINGER         0x02
#define FINGERPRINT_IMAGEFAIL        0x03
#define FINGERPRINT_IMAGEMESS        0x06
#define FINGERPRINT_FEATUREFAIL      0x07
#define FINGERPRINT_NOTFOUND         0x09
#define FINGERPRINT_ENROLLMISMATCH   0x0A
#define FINGERPRINT_BADLOCATION      0x0B
#define FINGERPRINT_DELETEFAIL       0x10
#define FINGERPRINT_INVALIDIMAGE     0x15
#define FINGERPRINT_FLASHERR         0x18

typedef struct {
    uint16_t start_code;
    uint32_t address;
    uint8_t  type;
    uint16_t length;
    uint8_t  data[64];
    uint16_t checksum;
} F_Packet;

extern F_Packet fpacket;
extern F_Packet rpacket;
extern uint16_t capacity;
extern uint16_t baud_rate;

void    fp_uart_start_it(void);
void    fp_uart_irq_handler(void);

/* Low-level */
void    init_fingerprint(void);
void    setup_packet(const uint8_t *data, uint8_t size);
void    setup_received(const uint8_t *data);
void    fp_send_packet(void);
void    fp_receive_packet_timeout(uint32_t timeout_ms);
uint8_t fp_txrx(void);
uint8_t fp_txrx_timeout(uint32_t timeout_ms);

/* Helpers */
uint8_t  fp_get_image(void);
uint8_t  fp_image2tz(uint8_t slot);
void     fp_wait_finger_removed(void);
uint8_t  fp_wait_finger_removed_timeout(uint32_t timeout_ms);
uint8_t  fp_create_model(void);
uint8_t  fp_store_model(uint16_t id);
uint8_t  fp_delete_model(uint16_t id);
uint8_t  fp_search_slot(uint8_t slot, uint16_t *match_id);
uint8_t  fp_get_template_index(uint8_t page, uint8_t *index_buf);
uint16_t fp_find_empty_id(void);
/* High-level */
uint8_t  save_fingerprint(uint16_t id);
uint16_t check_fingerprint(void);
void     led_mode(uint8_t control);
uint16_t get_template_number(void);
void     reset_database(void);

#ifdef __cplusplus
}
#endif

#endif /* __FINGERPRINT_H */
