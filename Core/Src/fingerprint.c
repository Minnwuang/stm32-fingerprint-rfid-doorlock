#include "fingerprint.h"
#include <string.h>

#ifdef USE_FREERTOS
  #include "cmsis_os2.h"
  #include "FreeRTOS.h"
  #include "task.h"
  extern osMutexId_t fpMutexHandle;
  extern osSemaphoreId_t fpRxSemHandle;
#endif
#ifdef USE_FREERTOS
static void fp_rx_sem_drain(void);
#endif

/* ====================== RING BUFFER ====================== */
#define FP_RING_SIZE  128U

static uint8_t           fp_ring_buf[FP_RING_SIZE];
static volatile uint16_t fp_ring_head = 0U;  /* ISR ghi */
static volatile uint16_t fp_ring_tail = 0U;  /* Task đọc */

/*
 * fp_uart_start_it:
 *   Bật RXNE và ERR interrupt trực tiếp qua register.
 *   KHÔNG gọi HAL_UART_Receive_IT — tránh hoàn toàn vấn đề lock.
 *   Interrupt RXNE luôn active, tự động nhận mọi byte từ module.
 *
 *   Gọi 1 lần duy nhất, trước init_fingerprint().
 *   Đảm bảo USART3_IRQHandler và NVIC đã được enable trước khi gọi hàm này.
 */
void fp_uart_start_it(void)
{
    fp_ring_head = 0U;
    fp_ring_tail = 0U;

    /* Clear error flags trước (đọc SR → DR) */
    volatile uint32_t tmp_sr = UART_HANDLER.Instance->SR;
    volatile uint32_t tmp_dr = UART_HANDLER.Instance->DR;
    (void)tmp_sr;
    (void)tmp_dr;

    /* Bật RXNE interrupt (CR1 bit 5) — luôn active, không phải one-shot */
    UART_HANDLER.Instance->CR1 |= USART_CR1_RXNEIE;

    /* Bật Error interrupt (CR3 bit 0) — bắt ORE, FE, NE */
    UART_HANDLER.Instance->CR3 |= USART_CR3_EIE;
}

/*
 * fp_uart_irq_handler:
 *   Đặt lời gọi hàm này vào USART3_IRQHandler trong stm32f1xx_it.c:
 *
 *     void USART3_IRQHandler(void) { fp_uart_irq_handler(); }
 *
 *   Xử lý RXNE: đọc byte từ DR và lưu vào ring buffer.
 *   Xử lý lỗi: clear ORE/FE/NE bằng cách đọc SR → DR.
 *   Không có HAL lock → không bao giờ bị chặn.
 */
void fp_uart_irq_handler(void)
{
    uint32_t sr = UART_HANDLER.Instance->SR;

    /* ---- RXNE: có byte mới ---- */
    if (sr & USART_SR_RXNE)
    {
        uint8_t byte = (uint8_t)(UART_HANDLER.Instance->DR & 0xFFU);
        uint16_t next = (uint16_t)((fp_ring_head + 1U) % FP_RING_SIZE);

        if (next != fp_ring_tail)
        {
#ifdef USE_FREERTOS
            uint8_t was_empty = (fp_ring_head == fp_ring_tail) ? 1U : 0U;
#endif
            fp_ring_buf[fp_ring_head] = byte;
            fp_ring_head = next;

#ifdef USE_FREERTOS
            if (was_empty && fpRxSemHandle != NULL)
            {
                osSemaphoreRelease(fpRxSemHandle);
            }
#endif
        }
    }
    else if (sr & (USART_SR_ORE | USART_SR_FE | USART_SR_NE))
    {
        volatile uint32_t dummy = UART_HANDLER.Instance->DR;
        (void)dummy;
    }
}

/* ---- Flush ring buffer (gọi trước khi gửi lệnh mới) ---- */
static void fp_ring_flush(void)
{
    fp_ring_tail = fp_ring_head;
#ifdef USE_FREERTOS
    fp_rx_sem_drain();
#endif
}

/*
 * fp_ring_read_byte:
 *   Đọc 1 byte từ ring buffer, chờ tối đa timeout_ms.
 *   osDelay(1) để yield trong RTOS, ISR vẫn chạy độc lập và nạp bytes.
 *   Trả về 0 = OK, 1 = timeout.
 */
static uint8_t fp_ring_read_byte(uint8_t *byte, uint32_t timeout_ms)
{
    uint32_t t0 = HAL_GetTick();

    if (byte == NULL)
        return 1U;

    while (1)
    {
        if (fp_ring_head != fp_ring_tail)
        {
            *byte = fp_ring_buf[fp_ring_tail];
            fp_ring_tail = (uint16_t)((fp_ring_tail + 1U) % FP_RING_SIZE);
            return 0U;
        }

        if ((HAL_GetTick() - t0) >= timeout_ms)
            return 1U;

#ifdef USE_FREERTOS
        if (fpRxSemHandle != NULL)
        {
            uint32_t elapsed = HAL_GetTick() - t0;
            uint32_t remain  = (elapsed < timeout_ms) ? (timeout_ms - elapsed) : 0U;

            if (remain == 0U)
                return 1U;

            (void)osSemaphoreAcquire(fpRxSemHandle, remain);
        }
        else
        {
            osDelay(1U);
        }
#else
        HAL_Delay(1U);
#endif
    }
}
#ifdef USE_FREERTOS
static void fp_rx_sem_drain(void)
{
    if (fpRxSemHandle != NULL)
    {
        while (osSemaphoreAcquire(fpRxSemHandle, 0U) == osOK) {}
    }
}
#endif
/* ====================== GLOBALS ====================== */
F_Packet fpacket = {
    .start_code = FINGERPRINT_STARTCODE,
    .address    = 0xFFFFFFFFUL,
    .type       = FINGERPRINT_COMMANDPACKET,
};

F_Packet rpacket;

uint16_t capacity  = FINGER_CAPACITY;
uint16_t baud_rate = 57600U;

/* ====================== INIT ====================== */
void init_fingerprint(void)
{
    uint8_t cmd = FINGERPRINT_READSYSPARAM;
    setup_packet(&cmd, 1);

    if (fp_txrx() == FINGERPRINT_OK)
    {
        uint16_t cap  = ((uint16_t)rpacket.data[5] << 8) | (uint16_t)rpacket.data[6];
        uint16_t baud = ((uint16_t)rpacket.data[7] << 8) | (uint16_t)rpacket.data[8];
        capacity  = (cap  != 0U) ? cap  : FINGER_CAPACITY;
        baud_rate = (baud != 0U) ? baud : 57600U;
    }
    else
    {
        capacity  = FINGER_CAPACITY;
        baud_rate = 57600U;
    }
}

/* ====================== PACKET BUILD ====================== */
void setup_packet(const uint8_t *data, uint8_t size)
{
    if ((data == NULL) || (size > sizeof(fpacket.data))) return;

    memset(fpacket.data, 0, sizeof(fpacket.data));
    fpacket.length = size;

    {
        uint16_t packet_length = (uint16_t)size + 2U;
        uint16_t sum = ((packet_length >> 8) & 0xFFU) +
                       (packet_length & 0xFFU) +
                       fpacket.type;
        for (uint8_t i = 0; i < size; i++) {
            fpacket.data[i] = data[i];
            sum = (uint16_t)(sum + data[i]);
        }
        fpacket.checksum = sum;
    }
}

void setup_received(const uint8_t *data)
{
    uint16_t data_len = 0U;

    if (data == NULL) {
        memset(&rpacket, 0, sizeof(rpacket));
        rpacket.data[0] = FINGERPRINT_PACKETRECIEVEERR;
        return;
    }

    rpacket.start_code = ((uint16_t)data[0] << 8) | (uint16_t)data[1];
    rpacket.address    = ((uint32_t)data[2] << 24) |
                         ((uint32_t)data[3] << 16) |
                         ((uint32_t)data[4] << 8)  |
                         ((uint32_t)data[5]);
    rpacket.type   = data[6];
    rpacket.length = ((uint16_t)data[7] << 8) | (uint16_t)data[8];

    if (rpacket.length >= 2U)
        data_len = (uint16_t)(rpacket.length - 2U);
    if (data_len > sizeof(rpacket.data))
        data_len = sizeof(rpacket.data);

    memset(rpacket.data, 0, sizeof(rpacket.data));
    for (uint16_t i = 0; i < data_len; i++)
        rpacket.data[i] = data[9U + i];

    if (rpacket.length >= 2U) {
        rpacket.checksum = ((uint16_t)data[9U + data_len] << 8) |
                           (uint16_t)data[10U + data_len];
    } else {
        rpacket.checksum = 0U;
    }
}

/* ====================== SEND ====================== */
/*
 * fp_send_packet: Ghi trực tiếp vào USART DR register.
 *
 * KHÔNG dùng HAL_UART_Transmit vì hàm đó acquire __HAL_LOCK(huart).
 * Nếu lock bị giữ trong khi ISR cố gọi HAL_UART_Receive_IT
 * → IT không được re-arm → 0x01.
 * (Cách cũ đã gây ra bug này.)
 *
 * Viết trực tiếp vào DR: poll TXE flag rồi ghi. Đơn giản, không lock.
 */
void fp_send_packet(void)
{
    uint16_t packet_length = (uint16_t)(fpacket.length + 2U);

    /* Macro gửi 1 byte: chờ TXE rồi ghi DR */
    #define FP_TX_BYTE(b) \
        do { \
            while (!(UART_HANDLER.Instance->SR & USART_SR_TXE)) {} \
            UART_HANDLER.Instance->DR = (b); \
        } while(0)

    /* Start code (2 bytes) */
    FP_TX_BYTE((uint8_t)(fpacket.start_code >> 8));
    FP_TX_BYTE((uint8_t)(fpacket.start_code & 0xFF));

    /* Address 0xFFFFFFFF (4 bytes) */
    FP_TX_BYTE(0xFF); FP_TX_BYTE(0xFF);
    FP_TX_BYTE(0xFF); FP_TX_BYTE(0xFF);

    /* Packet type */
    FP_TX_BYTE(fpacket.type);

    /* Length (2 bytes) */
    FP_TX_BYTE((uint8_t)(packet_length >> 8));
    FP_TX_BYTE((uint8_t)(packet_length & 0xFF));

    /* Data */
    for (uint16_t i = 0; i < fpacket.length; i++)
        FP_TX_BYTE(fpacket.data[i]);

    /* Checksum (2 bytes) */
    FP_TX_BYTE((uint8_t)(fpacket.checksum >> 8));
    FP_TX_BYTE((uint8_t)(fpacket.checksum & 0xFF));

    /* Chờ TX hoàn tất (TC flag) trước khi trả về */
    while (!(UART_HANDLER.Instance->SR & USART_SR_TC)) {}

    #undef FP_TX_BYTE
}

/* ====================== RECEIVE ====================== */
void fp_receive_packet_timeout(uint32_t timeout_ms)
{
    uint8_t  hdr[9]   = {0};
    uint8_t  tail[66] = {0};  /* max data(64) + checksum(2) */
    uint8_t  full[75] = {0};
    uint16_t len      = 0U;

    /* Đọc 9 byte header */
    for (uint8_t i = 0; i < 9U; i++) {
        if (fp_ring_read_byte(&hdr[i], timeout_ms) != 0U) {
            memset(&rpacket, 0, sizeof(rpacket));
            rpacket.data[0] = FINGERPRINT_PACKETRECIEVEERR;
            return;
        }
    }

    len = ((uint16_t)hdr[7] << 8) | (uint16_t)hdr[8];
    if ((len == 0U) || (len > sizeof(tail))) {
        memset(&rpacket, 0, sizeof(rpacket));
        rpacket.data[0] = FINGERPRINT_PACKETRECIEVEERR;
        return;
    }

    /* Đọc len byte (data + checksum) */
    for (uint16_t i = 0U; i < len; i++) {
        if (fp_ring_read_byte(&tail[i], timeout_ms) != 0U) {
            memset(&rpacket, 0, sizeof(rpacket));
            rpacket.data[0] = FINGERPRINT_PACKETRECIEVEERR;
            return;
        }
    }

    memcpy(full, hdr, 9U);
    memcpy(full + 9U, tail, len);
    setup_received(full);

    if (rpacket.start_code != FINGERPRINT_STARTCODE) {
        memset(&rpacket, 0, sizeof(rpacket));
        rpacket.data[0] = FINGERPRINT_PACKETRECIEVEERR;
        return;
    }
    if (rpacket.type != FINGERPRINT_ACKPACKET) {
        memset(&rpacket, 0, sizeof(rpacket));
        rpacket.data[0] = FINGERPRINT_PACKETRECIEVEERR;
        return;
    }
}

/* ====================== TXRX ====================== */
uint8_t fp_txrx_timeout(uint32_t timeout_ms)
{
#ifdef USE_FREERTOS
    osMutexAcquire(fpMutexHandle, osWaitForever);
#endif

    /*
     * Flush ring buffer trước khi gửi lệnh.
     * Xóa byte thừa (noise, echo lần trước) khỏi buffer.
     * An toàn vì ISR tiếp tục nhận byte mới sau flush.
     */
    fp_ring_flush();

    fp_send_packet();
    fp_receive_packet_timeout(timeout_ms);
    uint8_t result = rpacket.data[0];

#ifdef USE_FREERTOS
    osMutexRelease(fpMutexHandle);
#endif

    return result;
}

uint8_t fp_txrx(void)
{
    return fp_txrx_timeout(DEFAULTTIMEOUT);
}

/* ====================== COMMAND HELPERS ====================== */
uint8_t fp_get_image(void)
{
    uint8_t cmd = FINGERPRINT_GETIMAGE;
    setup_packet(&cmd, 1);
    return fp_txrx();
}

uint8_t fp_image2tz(uint8_t slot)
{
    uint8_t data[2] = { FINGERPRINT_IMAGE2TZ, slot };
    setup_packet(data, 2);
    return fp_txrx();
}

void fp_wait_finger_removed(void)
{
    while (fp_wait_finger_removed_timeout(10000U) == 0U)
        FP_DELAY(50);
}

uint8_t fp_wait_finger_removed_timeout(uint32_t timeout_ms)
{
    uint32_t t0          = HAL_GetTick();
    uint8_t  noFingerCnt = 0U;

    while ((HAL_GetTick() - t0) < timeout_ms)
    {
        uint8_t res = fp_get_image();
        if (res == FINGERPRINT_NOFINGER) {
            noFingerCnt++;
            if (noFingerCnt >= 3U) return 1U;
        } else {
            noFingerCnt = 0U;
        }
        FP_DELAY(80);
    }
    return 0U;
}

/*
 * fp_create_model: MODELTIMEOUT = 3000ms.
 * Module cần ~1.5–2s để tính toán tạo template từ 2 ảnh.
 */
uint8_t fp_create_model(void)
{
    uint8_t cmd = FINGERPRINT_REGMODEL;
    setup_packet(&cmd, 1);
    return fp_txrx_timeout(MODELTIMEOUT);
}

uint8_t fp_store_model(uint16_t id)
{
    uint8_t data[4] = {
        FINGERPRINT_STORE, 0x01U,
        (uint8_t)(id >> 8), (uint8_t)(id & 0xFFU)
    };
    setup_packet(data, 4);
    return fp_txrx();
}

uint8_t fp_delete_model(uint16_t id)
{
    uint8_t data[5] = {
        FINGERPRINT_DELETE,
        (uint8_t)(id >> 8), (uint8_t)(id & 0xFFU),
        0x00U, 0x01U
    };
    setup_packet(data, 5);
    return fp_txrx();
}

uint8_t fp_search_slot(uint8_t slot, uint16_t *match_id)
{
    uint8_t data[6] = {
        FINGERPRINT_SEARCH, slot,
        0x00U, 0x00U,
        (uint8_t)(capacity >> 8), (uint8_t)(capacity & 0xFFU)
    };
    uint8_t res;

    setup_packet(data, 6);
    res = fp_txrx();

    if (res == FINGERPRINT_OK) {
        if (match_id != NULL)
            *match_id = ((uint16_t)rpacket.data[1] << 8) | (uint16_t)rpacket.data[2];
        return FINGERPRINT_OK;
    }
    if (res == FINGERPRINT_NOTFOUND) return FINGERPRINT_NOTFOUND;
    return res;
}

/* ====================== HIGH LEVEL ====================== */
uint8_t save_fingerprint(uint16_t id)
{
    uint8_t res;

    /*
     * DEBUG MAP TẠM:
     * 0x30 = lỗi GETIMAGE lần 1
     * 0x31 = lỗi IMAGE2TZ(1)
     * 0x32 = timeout chờ nhấc ngón
     * 0x33 = lỗi GETIMAGE lần 2
     * 0x34 = lỗi IMAGE2TZ(2)
     * 0x35 = lỗi REGMODEL
     * 0x36 = lỗi STORE
     */

    /* ---- Lần 1: chờ đặt ngón ---- */
    while (1)
    {
        res = fp_get_image();
        if (res == FINGERPRINT_NOFINGER)
        {
            FP_DELAY(50);
            continue;
        }
        break;
    }

    if (res != FINGERPRINT_OK)
        return 0x30;   /* DEBUG: fail ở GETIMAGE lần 1 */

    /* Không delay thừa giữa GETIMAGE OK và IMAGE2TZ */
    res = fp_image2tz(1);
    if (res != FINGERPRINT_OK)
        return 0x31;   /* DEBUG: fail ở IMAGE2TZ(1) */

    FP_DELAY(200);

    /* ---- Chờ nhấc ngón ---- */
    if (!fp_wait_finger_removed_timeout(5000U))
        return 0x32;   /* DEBUG: timeout chờ nhấc ngón */

    FP_DELAY(200);

    /* ---- Lần 2: chờ đặt ngón lại ---- */
    while (1)
    {
        res = fp_get_image();
        if (res == FINGERPRINT_NOFINGER)
        {
            FP_DELAY(50);
            continue;
        }
        break;
    }

    if (res != FINGERPRINT_OK)
        return 0x33;   /* DEBUG: fail ở GETIMAGE lần 2 */

    res = fp_image2tz(2);
    if (res != FINGERPRINT_OK)
        return 0x34;   /* DEBUG: fail ở IMAGE2TZ(2) */

    FP_DELAY(200);

    /* ---- Tạo model ---- */
    res = fp_create_model();
    if (res != FINGERPRINT_OK)
        return 0x35;   /* DEBUG: fail ở REGMODEL */

    FP_DELAY(100);

    /* ---- Lưu model ---- */
    res = fp_store_model(id);
    if (res != FINGERPRINT_OK)
        return 0x36;   /* DEBUG: fail ở STORE */

    return FINGERPRINT_OK;
}
/*
 * check_fingerprint: TỰ GỌI fp_get_image() bên trong.
 * FingerTask KHÔNG được pre-call fp_get_image() trước hàm này.
 * Trả về: ID khớp / 0xFFFE = not found / 0xFFFF = error.
 */
uint16_t check_fingerprint(void)
{
    uint8_t res;
    uint8_t data[6] = {
        FINGERPRINT_SEARCH, 0x02U,
        0x00U, 0x00U,
        (uint8_t)(capacity >> 8), (uint8_t)(capacity & 0xFFU)
    };

    res = fp_get_image();
    if (res != FINGERPRINT_OK) return 0xFFFFU;

    res = fp_image2tz(2);
    if (res != FINGERPRINT_OK) return 0xFFFFU;

    setup_packet(data, 6);
    res = fp_txrx();

    if (res == FINGERPRINT_OK)
        return ((uint16_t)rpacket.data[1] << 8) | (uint16_t)rpacket.data[2];
    if (res == FINGERPRINT_NOTFOUND) return 0xFFFEU;
    return 0xFFFFU;
}

void led_mode(uint8_t control)
{
    uint8_t cmd;
    if ((control != 0U) && (control != 1U)) return;
    cmd = (control == 0U) ? FINGERPRINT_LEDOFF : FINGERPRINT_LEDON;
    setup_packet(&cmd, 1);
    (void)fp_txrx();
}

uint16_t get_template_number(void)
{
    uint8_t cmd = FINGERPRINT_TEMPLATECOUNT;
    setup_packet(&cmd, 1);
    if (fp_txrx() != FINGERPRINT_OK) return 0U;
    return ((uint16_t)rpacket.data[1] << 8) | (uint16_t)rpacket.data[2];
}
uint8_t fp_get_template_index(uint8_t page, uint8_t *index_buf)
{
    uint8_t data[2] = { FINGERPRINT_TEMPLATEINDEX, page };

    if (index_buf == NULL)
        return FINGERPRINT_PACKETRECIEVEERR;

    setup_packet(data, 2);

    if (fp_txrx() != FINGERPRINT_OK)
        return FINGERPRINT_PACKETRECIEVEERR;

    /*
     * ACK code + 32 byte bitmap => packet phải đủ dài
     * data count = 33 bytes, length = 35 (kèm checksum 2 bytes)
     */
    if (rpacket.length < 35U)
        return FINGERPRINT_PACKETRECIEVEERR;

    memcpy(index_buf, &rpacket.data[1], 32);
    return FINGERPRINT_OK;
}

uint16_t fp_find_empty_id(void)
{
    uint8_t index_buf[32];

    if (fp_get_template_index(0, index_buf) != FINGERPRINT_OK)
        return 0xFFFFU;   /* lỗi đọc index */

    for (uint16_t byte = 0U; byte < 32U; byte++)
    {
        for (uint8_t bit = 0U; bit < 8U; bit++)
        {
            uint16_t id = (uint16_t)(byte * 8U + bit + 1U);

            if (id > capacity)
                return 0U;   /* full */

            /* QUAN TRỌNG: quét bit từ trái sang phải (MSB -> LSB) */
            if ((index_buf[byte] & (1U << (7U - bit))) == 0U)
                return id;
        }
    }

    return 0U;
}
void reset_database(void)
{
    uint8_t cmd = FINGERPRINT_EMPTY;
    setup_packet(&cmd, 1);
    (void)fp_txrx();
}
