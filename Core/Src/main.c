/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c  (PART 1 / 2)
  * @brief          : AS608 + RC522 + LCD 16x2 I2C + HC-SR04 + Servo + FreeRTOS
  *
  * Hardware:
  *   I2C1   : PB6(SCL), PB7(SDA)  -> LCD 16x2 PCF8574
  *   SPI1   : PA5/PA6/PA7         -> RC522
  *   USART3 : PB10(TX), PB11(RX)  -> AS608 fingerprint
  *   TIM2   : PA0 (IC CH1)        -> HC-SR04 Echo
  *   TIM3   : PB1 (PWM CH4)       -> Servo
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include "CLCD_I2C.h"
#include "RC522.h"
#include "fingerprint.h"
#include "string.h"
#include "stdio.h"
#include "stdbool.h"
#include "stdint.h"
#include "cmsis_os2.h"

/* ====================== PRIVATE VARIABLES ====================== */
I2C_HandleTypeDef  hi2c1;
SPI_HandleTypeDef  hspi1;
UART_HandleTypeDef huart3;   /* AS608 fingerprint sensor */
TIM_HandleTypeDef  htim2;
TIM_HandleTypeDef  htim3;
CLCD_I2C_Name      LCD1;

/* RC522 */
uint8_t status;
uint8_t str[16];
uint8_t sNum[5];

/* ====================== CONFIG ====================== */
#define LCD_LINE_LEN            16U
#define PASSWORD_MIN_LEN        2U
#define PASSWORD_MAX_LEN        16U
#define MAX_USERS               10U
#define UID_LEN                 4U
#define USER_EXTEND_WINDOW_SEC  10U
#define EXTEND_MINUTES_MAX      99999UL
#define KEYPAD_SCAN_PERIOD_MS   20U
#define RFID_SCAN_PERIOD_MS     50U
#define AUTO_SCAN_PERIOD_MS     120U
#define AUTO_ECHO_WAIT_MS       60U
#define AUTO_TRIGGER_CM_X10     500UL
#define HCSR04_TRIG_Pin         GPIO_PIN_1
#define HCSR04_TRIG_GPIO_Port   GPIOA
#define DOOR_SW_Pin             GPIO_PIN_2
#define DOOR_SW_GPIO_Port       GPIOA
#define SERVO_CCR_0_DEG         25U
#define SERVO_CCR_90_DEG        75U
#define SERVO_CCR_180_DEG       125U
#define DOOR_CLOSE_ANGLE        0U
#define DOOR_OPEN_ANGLE         90U
#define WAIT_CLOSE_RETRY_SEC    3UL

/* ====================== APP STATE ====================== */
typedef enum
{
    ST_LOGIN = 0,
    ST_ADMIN_MENU,
    ST_ADMIN_ADD_MENU,
    ST_ADMIN_DELETE_MENU,
    ST_ADMIN_ADD_CARD_WAIT,
    ST_ADMIN_DELETE_CARD_WAIT,
    ST_ADMIN_ADD_FP_WAIT,       /* Chờ đặt ngón để đăng ký vân tay  */
    ST_ADMIN_DELETE_FP_WAIT,    /* Chờ đặt ngón để xóa vân tay      */
    ST_CHANGE_PW_MENU,
    ST_CHANGE_ADMIN_OLD,
    ST_CHANGE_ADMIN_NEW,
    ST_CHANGE_USER_OLD,
    ST_CHANGE_USER_NEW,
    ST_ADMIN_MENU_2,
    ST_AUTO_ARMED,
    ST_USER_EXTEND_WAIT,
    ST_USER_OPEN_HOLD,
    ST_WAIT_DOOR_CLOSE
} AppState;

/* ====================== EVENT TYPES ====================== */
typedef enum
{
    EVT_KEY = 0,
    EVT_CARD,
    EVT_SECOND_TICK,
    EVT_UI_TEMP_TIMEOUT,
    EVT_ALARM_TIMEOUT,
    EVT_AUTO_NEAR,
    EVT_DOOR_CLOSED,
    EVT_FP_MATCH,        /* fp_id = ID khớp                */
    EVT_FP_NOTFOUND,     /* không tìm thấy                 */
    EVT_FP_ERROR,        /* lỗi scan / giao tiếp           */
    EVT_FP_ENROLLED,     /* fp_id = ID vừa đăng ký         */
    EVT_FP_ENROLL_FAIL,  /* fp_id = mã lỗi từ module       */
    EVT_FP_DELETED,      /* fp_id = ID vừa xóa             */
} AppEventType;

typedef struct
{
    AppEventType type;
    char         key;
    uint8_t      uid[UID_LEN];
    uint16_t     fp_id;   /* dùng cho fingerprint events    */
} AppEvent;

typedef enum
{
    CLOSE_TARGET_NORMAL = 0,
    CLOSE_TARGET_AUTO_ARMED
} CloseTarget;

/* ====================== LCD MESSAGE ====================== */
typedef struct
{
    char line1[LCD_LINE_LEN + 1];
    char line2[LCD_LINE_LEN + 1];
} LcdMessage;

/* ====================== RTOS HANDLES ====================== */
osThreadId_t keypadTaskHandle;
osThreadId_t rfidTaskHandle;
osThreadId_t accessTaskHandle;
osThreadId_t lcdTaskHandle;
osThreadId_t autoTaskHandle;
osThreadId_t fingerTaskHandle;    /* AS608 fingerprint task */

osMessageQueueId_t appEventQueueHandle;
osMessageQueueId_t lcdQueueHandle;

osTimerId_t secondTimerHandle;
osTimerId_t uiTempTimerHandle;
osTimerId_t alarmTimerHandle;

/*
 * lcdMutexHandle : bảo vệ bus I2C của LCD khỏi bị nhiều task ghi đồng thời.
 * fpMutexHandle  : bảo vệ fpacket/rpacket của fingerprint.
 *                  fp_txrx() trong fingerprint.c acquire/release mutex này.
 */
osMutexId_t lcdMutexHandle;
osMutexId_t fpMutexHandle;

/* ====================== APP DATA ====================== */
AppState app_state = ST_LOGIN;

char admin_password[PASSWORD_MAX_LEN + 1]   = "000";
char user_password[PASSWORD_MAX_LEN + 1]    = "111";
char pending_welcome_line[LCD_LINE_LEN + 1] = {0};
char input_buffer[PASSWORD_MAX_LEN + 1]     = {0};
uint8_t input_len = 0U;

uint8_t registered_cards[MAX_USERS][UID_LEN] =
{
    {0xD0, 0x9E, 0xEB, 0x5F},
    {0x55, 0x2E, 0x0F, 0x06},
    {0x43, 0x60, 0x8B, 0x1B}
};
uint8_t card_valid[MAX_USERS] = {1, 1, 1};

uint8_t  wrong_pass_count          = 0U;
bool     ui_temp_active            = false;
bool     pending_user_session      = false;
bool     auto_mode_enabled         = false;
uint32_t auto_mode_seconds_left    = 0UL;
uint32_t extend_seconds_left       = 0UL;
uint32_t door_remaining_seconds    = 0UL;
CloseTarget close_target_after_wait  = CLOSE_TARGET_NORMAL;
uint32_t    wait_close_retry_seconds = 0UL;

/* HC-SR04 */
volatile uint32_t ic_val1           = 0UL;
volatile uint32_t ic_val2           = 0UL;
volatile uint32_t diff_capture      = 0UL;
volatile uint32_t distance_x10_cm   = 0UL;
volatile uint8_t  is_first_captured = 0U;
volatile uint8_t  distance_ready    = 0U;
/* ====================== FP ID MAP (RAM + FLASH) ====================== */
static uint8_t  fp_id_used[FINGER_CAPACITY + 1] = {0};  /* dùng ID 1..127 */
static uint16_t fp_next_hint = 1U;

#define FP_FLASH_PAGE_ADDR   0x0801FC00UL   /* trang flash cuối cho STM32F103xB 128KB */
#define FP_FLASH_MAGIC       0x46504944UL   /* 'FPID' */
#define FP_FLASH_VERSION     0x0001U

typedef struct
{
    uint32_t magic;
    uint16_t version;
    uint16_t checksum;
    uint8_t  used[FINGER_CAPACITY + 1];
    uint8_t  reserved[2];
} FpIdMapRecord;
#define CARD_FLASH_PAGE_ADDR  0x0801F800UL   /* trang kế cuối */
#define CARD_FLASH_MAGIC      0x43415244UL   /* 'CARD' */
#define CARD_FLASH_VERSION    0x0001U

typedef struct
{
    uint32_t magic;
    uint16_t version;
    uint16_t checksum;
    uint8_t  registered_cards[MAX_USERS][UID_LEN];
    uint8_t  card_valid[MAX_USERS];
    uint8_t  reserved[1];
} CardFlashRecord;
/* ====================== PROTOTYPES ====================== */
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART3_UART_Init(void);
void Servo_SetAngle(uint8_t angle);
void Door_Open(void);
void Door_Close(void);
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
void HCSR04_Trigger(void);
void HCSR04_TrigPulse(void);
bool DoorSensor_IsClosed(void);
void BeginUserWelcome(const char *welcome_line2);
void BeginWaitDoorClose(CloseTarget target);
void FinishCloseSequence(void);
void HandleObstacleDetected(void);
void EnterAutoMode(void);
void ExitAutoModeImmediate(void);
void StartAutoDetectedSession(void);
void CloseDoorToAutoArmed(void);
void StartUserExtendPhase(void);
void StartKeypadTask(void *argument);
void StartRFIDTask(void *argument);
void StartAccessControlTask(void *argument);
void StartLCDTask(void *argument);
void StartAutoTask(void *argument);
void StartFingerTask(void *argument);
void SecondTimerCallback(void *argument);
void UiTempTimerCallback(void *argument);
void AlarmTimerCallback(void *argument);
void LCD_SendLines(const char *line1, const char *line2);
void LCD_WritePaddedLine(uint8_t row, const char *text);
void CopyLine(char *dst, const char *src);
void ClearInput(void);
void AppendInput(char key);
void BackspaceInput(void);
bool IsValidPasswordLength(uint8_t len);
void ShowCurrentScreen(void);
void ForceShowCurrentScreen(void);
void ShowTempOverlay(const char *line1, const char *line2, uint32_t duration_ms);
void ReturnToLogin(void);
void StartUserOpenSession(void);
void ApplyUserExtension(void);
void CloseDoorNow(void);
void HandleAppEvent(const AppEvent *evt);
void HandleKeyEvent(char key);
void HandleCardEvent(uint8_t *uid);
void BuildUserMessage(char *dst, const char *prefix, int slot);
void BuildUserTag(char *dst, int slot);
void BuildExtendLine(char *dst);
void BuildDoorOpenLine(char *dst);
void BuildAutoModeLine(char *dst);
void BuildWaitCloseLine(char *dst);
bool InputIsAutoModeMinutes(void);
uint32_t ParseAutoModeMinutes(const char *s);
void ApplyAutoModeExtension(void);
void ApplyDoorHoldExtension(void);
uint8_t UID_Equals(uint8_t *uid1, uint8_t *uid2);
int FindCard(uint8_t *uid);
int FindEmptyCardSlot(void);
uint32_t ParseMinutesSafe(const char *s);
char Keypad_ScanRaw(void);
void PostKeyEvent(char key);
void PostCardEvent(uint8_t *uid);
void PostSimpleEvent(AppEventType type);
void PostFpEvent(AppEventType type, uint16_t fp_id);

void FP_IdMap_Clear(void);
void FP_MarkIdUsed(uint16_t id);
void FP_ReleaseId(uint16_t id);
uint16_t FP_AllocateEmptyId(void);
void FP_RebuildNextHint(void);
uint16_t FP_CalcChecksum(const uint8_t *data, uint32_t len);
HAL_StatusTypeDef FP_Flash_SaveMap(void);
uint8_t FP_Flash_LoadMap(void);
HAL_StatusTypeDef Card_Flash_Save(void);
uint8_t Card_Flash_Load(void);
/* ====================== LOW LEVEL LCD ====================== */
void LCD_WritePaddedLine(uint8_t row, const char *text)
{
    char    buf[LCD_LINE_LEN + 1];
    uint8_t i = 0U;

    for (i = 0U; i < LCD_LINE_LEN; i++) buf[i] = ' ';
    buf[LCD_LINE_LEN] = '\0';
    if (text != NULL)
        for (i = 0U; i < LCD_LINE_LEN && text[i] != '\0'; i++)
            buf[i] = text[i];

    osMutexAcquire(lcdMutexHandle, osWaitForever);
    CLCD_I2C_SetCursor(&LCD1, 0, row);
    CLCD_I2C_WriteString(&LCD1, buf);
    osMutexRelease(lcdMutexHandle);
}

void LCD_SendLines(const char *line1, const char *line2)
{
    LcdMessage msg;
    CopyLine(msg.line1, line1);
    CopyLine(msg.line2, line2);
    osMessageQueuePut(lcdQueueHandle, &msg, 0U, 0U);
}

/* ====================== HC-SR04 ====================== */
void HCSR04_TrigPulse(void)
{
    HAL_GPIO_WritePin(HCSR04_TRIG_GPIO_Port, HCSR04_TRIG_Pin, GPIO_PIN_RESET);
    for (volatile uint16_t i = 0U; i < 30U;  i++) { __NOP(); }
    HAL_GPIO_WritePin(HCSR04_TRIG_GPIO_Port, HCSR04_TRIG_Pin, GPIO_PIN_SET);
    for (volatile uint16_t i = 0U; i < 180U; i++) { __NOP(); }
    HAL_GPIO_WritePin(HCSR04_TRIG_GPIO_Port, HCSR04_TRIG_Pin, GPIO_PIN_RESET);
}

void HCSR04_Trigger(void)
{
    is_first_captured = 0U;
    distance_ready    = 0U;
    __HAL_TIM_SET_COUNTER(&htim2, 0U);
    __HAL_TIM_SET_CAPTUREPOLARITY(&htim2, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
    __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_CC1);
    HCSR04_TrigPulse();
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
    {
        if (is_first_captured == 0U)
        {
            ic_val1           = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
            is_first_captured = 1U;
            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1,
                                          TIM_INPUTCHANNELPOLARITY_FALLING);
        }
        else
        {
            ic_val2       = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
            diff_capture  = (ic_val2 >= ic_val1) ?
                            (ic_val2 - ic_val1) :
                            (0xFFFFUL - ic_val1 + ic_val2);
            distance_x10_cm   = (diff_capture * 343UL) / 2000UL;
            distance_ready    = 1U;
            is_first_captured = 0U;
            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1,
                                          TIM_INPUTCHANNELPOLARITY_RISING);
        }
    }
}

/* ====================== DOOR SENSOR ====================== */
bool DoorSensor_IsClosed(void)
{
    return (HAL_GPIO_ReadPin(DOOR_SW_GPIO_Port, DOOR_SW_Pin) == GPIO_PIN_RESET);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == DOOR_SW_Pin)
        if (DoorSensor_IsClosed())
            PostSimpleEvent(EVT_DOOR_CLOSED);
}

/* ====================== BASIC HELPERS ====================== */
void CopyLine(char *dst, const char *src)
{
    uint8_t i = 0U;
    if (dst == NULL) return;
    if (src == NULL) { dst[0] = '\0'; return; }
    while (src[i] != '\0' && i < LCD_LINE_LEN) { dst[i] = src[i]; i++; }
    dst[i] = '\0';
}

void ClearInput(void)
{
    memset(input_buffer, 0, sizeof(input_buffer));
    input_len = 0U;
}

void AppendInput(char key)
{
    if (input_len < PASSWORD_MAX_LEN)
    {
        input_buffer[input_len] = key;
        input_len++;
        input_buffer[input_len] = '\0';
    }
}

void BackspaceInput(void)
{
    if (input_len > 0U)
    {
        input_len--;
        input_buffer[input_len] = '\0';
    }
}

bool IsValidPasswordLength(uint8_t len)
{
    return (len >= PASSWORD_MIN_LEN && len <= PASSWORD_MAX_LEN);
}

uint32_t ParseMinutesSafe(const char *s)
{
    uint64_t value = 0ULL;
    uint8_t  i     = 0U;
    while (s[i] != '\0')
    {
        if (s[i] < '0' || s[i] > '9') break;
        value = value * 10ULL + (uint64_t)(s[i] - '0');
        if (value > EXTEND_MINUTES_MAX) { value = EXTEND_MINUTES_MAX; break; }
        i++;
    }
    return (uint32_t)value;
}

bool InputIsAutoModeMinutes(void)
{
    return (input_len >= 2U && (input_buffer[0] == 'A' || input_buffer[0] == 'a'));
}

uint32_t ParseAutoModeMinutes(const char *s)
{
    if (s == NULL) return 0UL;
    if (s[0] == 'A' || s[0] == 'a') return ParseMinutesSafe(&s[1]);
    return 0UL;
}

/* ====================== USER / CARD HELPERS ====================== */
void BuildUserTag(char *dst, int slot)
{
    uint8_t user_no;
    if (dst == NULL) return;
    if (slot < 0 || slot >= (int)MAX_USERS) { dst[0] = '\0'; return; }
    user_no = (uint8_t)(slot + 1);
    if (user_no < 10U)
        { dst[0] = 'U'; dst[1] = (char)('0' + user_no); dst[2] = '\0'; }
    else
    {
        dst[0] = 'U';
        dst[1] = (char)('0' + (user_no / 10U));
        dst[2] = (char)('0' + (user_no % 10U));
        dst[3] = '\0';
    }
}

void BuildUserMessage(char *dst, const char *prefix, int slot)
{
    char    tag[4];
    uint8_t i = 0U, j = 0U;
    if (dst == NULL) return;
    BuildUserTag(tag, slot);
    while (prefix != NULL && prefix[i] != '\0' && j < LCD_LINE_LEN) dst[j++] = prefix[i++];
    if (j < LCD_LINE_LEN) dst[j++] = ' ';
    i = 0U;
    while (tag[i] != '\0' && j < LCD_LINE_LEN) dst[j++] = tag[i++];
    dst[j] = '\0';
}

uint8_t UID_Equals(uint8_t *uid1, uint8_t *uid2)
{
    uint8_t i;
    for (i = 0U; i < UID_LEN; i++)
        if (uid1[i] != uid2[i]) return 0U;
    return 1U;
}

int FindCard(uint8_t *uid)
{
    int i;
    for (i = 0; i < (int)MAX_USERS; i++)
        if (card_valid[i] && UID_Equals(uid, registered_cards[i])) return i;
    return -1;
}

int FindEmptyCardSlot(void)
{
    int i;
    for (i = 0; i < (int)MAX_USERS; i++)
        if (card_valid[i] == 0U) return i;
    return -1;
}
void FP_IdMap_Clear(void)
{
    memset(fp_id_used, 0, sizeof(fp_id_used));
    fp_next_hint = 1U;
}

void FP_MarkIdUsed(uint16_t id)
{
    if (id >= 1U && id <= FINGER_CAPACITY)
    {
        fp_id_used[id] = 1U;
        fp_next_hint = id + 1U;
        if (fp_next_hint > FINGER_CAPACITY) fp_next_hint = 1U;
    }
}

void FP_ReleaseId(uint16_t id)
{
    if (id >= 1U && id <= FINGER_CAPACITY)
    {
        fp_id_used[id] = 0U;
        if (id < fp_next_hint) fp_next_hint = id;
    }
}

uint16_t FP_AllocateEmptyId(void)
{
    uint16_t start = fp_next_hint;
    uint16_t id    = start;

    do
    {
        if (id == 0U || id > FINGER_CAPACITY)
            id = 1U;

        if (fp_id_used[id] == 0U)
            return id;

        id++;
        if (id > FINGER_CAPACITY) id = 1U;

    } while (id != start);

    return 0U; /* full */
}
void FP_RebuildNextHint(void)
{
    fp_next_hint = 1U;
    for (uint16_t i = 1U; i <= FINGER_CAPACITY; i++)
    {
        if (fp_id_used[i] == 0U)
        {
            fp_next_hint = i;
            return;
        }
    }
}

uint16_t FP_CalcChecksum(const uint8_t *data, uint32_t len)
{
    uint32_t sum = 0U;
    for (uint32_t i = 0U; i < len; i++)
        sum += data[i];
    return (uint16_t)(sum & 0xFFFFU);
}

HAL_StatusTypeDef FP_Flash_SaveMap(void)
{
    FpIdMapRecord rec;
    FLASH_EraseInitTypeDef erase;
    uint32_t page_error = 0U;
    const uint8_t *p;
    HAL_StatusTypeDef st;

    memset(&rec, 0xFF, sizeof(rec));
    rec.magic    = FP_FLASH_MAGIC;
    rec.version  = FP_FLASH_VERSION;
    memcpy(rec.used, fp_id_used, sizeof(fp_id_used));
    rec.used[0]  = 0U;
    rec.checksum = FP_CalcChecksum(rec.used, sizeof(rec.used));

    HAL_FLASH_Unlock();

    memset(&erase, 0, sizeof(erase));
    erase.TypeErase   = FLASH_TYPEERASE_PAGES;
    erase.PageAddress = FP_FLASH_PAGE_ADDR;
    erase.NbPages     = 1U;

    st = HAL_FLASHEx_Erase(&erase, &page_error);
    if (st != HAL_OK)
    {
        HAL_FLASH_Lock();
        return st;
    }

    p = (const uint8_t *)&rec;
    for (uint32_t i = 0U; i < sizeof(rec); i += 2U)
    {
        uint16_t halfword = (uint16_t)p[i] | ((uint16_t)p[i + 1U] << 8);

        st = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,
                               FP_FLASH_PAGE_ADDR + i,
                               halfword);
        if (st != HAL_OK)
        {
            HAL_FLASH_Lock();
            return st;
        }
    }

    HAL_FLASH_Lock();
    return HAL_OK;
}

uint8_t FP_Flash_LoadMap(void)
{
    const FpIdMapRecord *rec = (const FpIdMapRecord *)FP_FLASH_PAGE_ADDR;

    if (rec->magic != FP_FLASH_MAGIC) return 0U;
    if (rec->version != FP_FLASH_VERSION) return 0U;
    if (rec->checksum != FP_CalcChecksum(rec->used, sizeof(rec->used))) return 0U;

    memcpy(fp_id_used, rec->used, sizeof(fp_id_used));
    fp_id_used[0] = 0U;
    FP_RebuildNextHint();
    return 1U;
}
HAL_StatusTypeDef Card_Flash_Save(void)
{
    CardFlashRecord rec;
    FLASH_EraseInitTypeDef erase;
    uint32_t page_error = 0U;
    const uint8_t *p;
    HAL_StatusTypeDef st;

    memset(&rec, 0xFF, sizeof(rec));
    rec.magic   = CARD_FLASH_MAGIC;
    rec.version = CARD_FLASH_VERSION;
    memcpy(rec.registered_cards, registered_cards, sizeof(registered_cards));
    memcpy(rec.card_valid, card_valid, sizeof(card_valid));

    rec.checksum = 0U;
    rec.checksum = FP_CalcChecksum((const uint8_t *)&rec.registered_cards[0][0],
                                   sizeof(rec.registered_cards))
                 + FP_CalcChecksum(rec.card_valid, sizeof(rec.card_valid));

    HAL_FLASH_Unlock();

    memset(&erase, 0, sizeof(erase));
    erase.TypeErase   = FLASH_TYPEERASE_PAGES;
    erase.PageAddress = CARD_FLASH_PAGE_ADDR;
    erase.NbPages     = 1U;

    st = HAL_FLASHEx_Erase(&erase, &page_error);
    if (st != HAL_OK)
    {
        HAL_FLASH_Lock();
        return st;
    }

    p = (const uint8_t *)&rec;
    for (uint32_t i = 0U; i < sizeof(rec); i += 2U)
    {
        uint16_t halfword = (uint16_t)p[i] | ((uint16_t)p[i + 1U] << 8);

        st = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,
                               CARD_FLASH_PAGE_ADDR + i,
                               halfword);
        if (st != HAL_OK)
        {
            HAL_FLASH_Lock();
            return st;
        }
    }

    HAL_FLASH_Lock();
    return HAL_OK;
}

uint8_t Card_Flash_Load(void)
{
    const CardFlashRecord *rec = (const CardFlashRecord *)CARD_FLASH_PAGE_ADDR;
    uint16_t checksum;

    if (rec->magic != CARD_FLASH_MAGIC) return 0U;
    if (rec->version != CARD_FLASH_VERSION) return 0U;

    checksum = FP_CalcChecksum((const uint8_t *)&rec->registered_cards[0][0],
                               sizeof(rec->registered_cards))
             + FP_CalcChecksum(rec->card_valid, sizeof(rec->card_valid));

    if (rec->checksum != checksum) return 0U;

    memcpy(registered_cards, rec->registered_cards, sizeof(registered_cards));
    memcpy(card_valid, rec->card_valid, sizeof(card_valid));
    return 1U;
}
/* ====================== DISPLAY BUILDERS ====================== */
static void AppendUint32(char *dst, uint8_t *j, uint32_t val, uint8_t minDigits)
{
    char    tmp[10];
    uint8_t k = 0U;
    if (val == 0UL) { tmp[k++] = '0'; }
    else
    {
        uint32_t v = val;
        while (v > 0UL && k < 9U) { tmp[k++] = (char)('0' + (v % 10UL)); v /= 10UL; }
    }
    while (k < minDigits && k < 9U) tmp[k++] = '0';
    uint8_t lo = 0U, hi = (uint8_t)(k - 1U);
    while (lo < hi) { char t = tmp[lo]; tmp[lo] = tmp[hi]; tmp[hi] = t; lo++; hi--; }
    uint8_t m = 0U;
    while (m < k && *j < LCD_LINE_LEN) dst[(*j)++] = tmp[m++];
}

void BuildExtendLine(char *dst)
{
    uint8_t j        = 0U;
    uint8_t sec_disp = (uint8_t)(extend_seconds_left > 99UL ? 99UL : extend_seconds_left);
    if (dst == NULL) return;
    if (input_len == 0U)
    {
        const char prefix[] = "Con ";
        uint8_t p = 0U;
        while (prefix[p] != '\0' && j < LCD_LINE_LEN) dst[j++] = prefix[p++];
        AppendUint32(dst, &j, sec_disp, 1U);
        if (j < LCD_LINE_LEN) dst[j++] = 's';
    }
    else
    {
        uint8_t i = 0U;
        while (input_buffer[i] != '\0' && i < 12U && j < LCD_LINE_LEN)
            dst[j++] = input_buffer[i++];
        if (j < LCD_LINE_LEN) dst[j++] = ' ';
        AppendUint32(dst, &j, sec_disp, 1U);
        if (j < LCD_LINE_LEN) dst[j++] = 's';
    }
    dst[j] = '\0';
}

void BuildDoorOpenLine(char *dst)
{
    uint8_t     j      = 0U;
    uint32_t    mins   = door_remaining_seconds / 60UL;
    uint32_t    secs   = door_remaining_seconds % 60UL;
    const char *suffix = auto_mode_enabled ? " D:thoat" : " D:dong";
    uint8_t     i      = 0U;
    if (dst == NULL) return;
    if (input_len > 0U)
    {
        while (input_buffer[i] != '\0' && j < LCD_LINE_LEN) dst[j++] = input_buffer[i++];
        if (j < LCD_LINE_LEN) dst[j++] = ' ';
        if (j < LCD_LINE_LEN) dst[j++] = '#';
        if (j < LCD_LINE_LEN) dst[j++] = '=';
        if (j < LCD_LINE_LEN) dst[j++] = 'O';
        if (j < LCD_LINE_LEN) dst[j++] = 'K';
        dst[j] = '\0';
        return;
    }
    if (mins > EXTEND_MINUTES_MAX) mins = EXTEND_MINUTES_MAX;
    AppendUint32(dst, &j, mins, 1U);
    if (j < LCD_LINE_LEN) dst[j++] = ':';
    AppendUint32(dst, &j, secs, 2U);
    while (suffix[i] != '\0' && j < LCD_LINE_LEN) dst[j++] = suffix[i++];
    dst[j] = '\0';
}

void BuildAutoModeLine(char *dst)
{
    uint8_t j = 0U;
    if (dst == NULL) return;
    if (input_len > 0U)
    {
        while (input_buffer[j] != '\0' && j < LCD_LINE_LEN)
            { dst[j] = input_buffer[j]; j++; }
        dst[j] = '\0';
        return;
    }
    if (auto_mode_seconds_left > 0UL)
    {
        AppendUint32(dst, &j, auto_mode_seconds_left / 60UL, 1U);
        if (j < LCD_LINE_LEN) dst[j++] = ':';
        AppendUint32(dst, &j, auto_mode_seconds_left % 60UL, 2U);
        if (j < LCD_LINE_LEN) dst[j++] = ' ';
    }
    {
        const char suffix[] = "C:Thoat";
        uint8_t    i        = 0U;
        while (suffix[i] != '\0' && j < LCD_LINE_LEN) dst[j++] = suffix[i++];
    }
    dst[j] = '\0';
}

void BuildWaitCloseLine(char *dst)
{
    uint8_t    j        = 0U;
    const char prefix[] = "Con ";
    const char suffix[] = "s dong cua";
    uint8_t    i        = 0U;
    if (dst == NULL) return;
    while (prefix[i] != '\0' && j < LCD_LINE_LEN) dst[j++] = prefix[i++];
    AppendUint32(dst, &j, wait_close_retry_seconds, 1U);
    i = 0U;
    while (suffix[i] != '\0' && j < LCD_LINE_LEN) dst[j++] = suffix[i++];
    dst[j] = '\0';
}

/* ====================== SCREEN CONTROL ====================== */
void ShowCurrentScreen(void)
{
    char line1[LCD_LINE_LEN + 1];
    char line2[LCD_LINE_LEN + 1];

    if (ui_temp_active) return;
    memset(line1, 0, sizeof(line1));
    memset(line2, 0, sizeof(line2));

    switch (app_state)
    {
        case ST_LOGIN:
            CopyLine(line1, "Nhap mat khau");
            CopyLine(line2, input_buffer);
            break;
        case ST_ADMIN_MENU:
            CopyLine(line1, "1Them  2Xoa");
            CopyLine(line2, "3DoiMK 4Next");
            break;
        case ST_ADMIN_ADD_MENU:
            CopyLine(line1, "Them:   1.Thetu");
            CopyLine(line2, "2.Vantay3.Thoat");
            break;
        case ST_ADMIN_DELETE_MENU:
            CopyLine(line1, "Xoa:    1.Thetu");
            CopyLine(line2, "2.Vantay3.Thoat");
            break;
        case ST_ADMIN_ADD_CARD_WAIT:
            CopyLine(line1, "Quet the moi");
            CopyLine(line2, "*:Thoat");
            break;
        case ST_ADMIN_DELETE_CARD_WAIT:
            CopyLine(line1, "Quet the xoa");
            CopyLine(line2, "*:Thoat");
            break;

        /* ---- fingerprint states ---- */
        case ST_ADMIN_ADD_FP_WAIT:
            CopyLine(line1, "Dat ngon 1...");
            CopyLine(line2, "*:Thoat");
            break;
        case ST_ADMIN_DELETE_FP_WAIT:
            CopyLine(line1, "Dat ngon xoa");
            CopyLine(line2, "*:Thoat");
            break;

        case ST_CHANGE_PW_MENU:
            CopyLine(line1, "DOIMK: 1.Admin");
            CopyLine(line2, "2.USER 3.Thoat");
            break;
        case ST_ADMIN_MENU_2:
            CopyLine(line1, "5.Mode Auto");
            CopyLine(line2, "4Back  6.Thoat");
            break;
        case ST_AUTO_ARMED:
            CopyLine(line1, "AUTO <50cm mo");
            BuildAutoModeLine(line2);
            break;
        case ST_CHANGE_ADMIN_OLD:
            CopyLine(line1, "MK cu Admin:");
            CopyLine(line2, input_buffer);
            break;
        case ST_CHANGE_ADMIN_NEW:
            CopyLine(line1, "MK moi Admin:");
            CopyLine(line2, input_buffer);
            break;
        case ST_CHANGE_USER_OLD:
            CopyLine(line1, "MK cu User:");
            CopyLine(line2, input_buffer);
            break;
        case ST_CHANGE_USER_NEW:
            CopyLine(line1, "MK moi User:");
            CopyLine(line2, input_buffer);
            break;
        case ST_USER_EXTEND_WAIT:
            CopyLine(line1, auto_mode_enabled ? "Giahan A+mode" : "Gia han so phut");
            BuildExtendLine(line2);
            break;
        case ST_USER_OPEN_HOLD:
            CopyLine(line1, auto_mode_enabled ? "AUTO cua dang mo" : "Cua dang mo");
            BuildDoorOpenLine(line2);
            break;
        case ST_WAIT_DOOR_CLOSE:
            CopyLine(line1, "Dang dong cua...");
            BuildWaitCloseLine(line2);
            break;
        default:
            CopyLine(line1, "System Ready");
            CopyLine(line2, "");
            break;
    }
    LCD_SendLines(line1, line2);
}

void ForceShowCurrentScreen(void)
{
    bool prev      = ui_temp_active;
    ui_temp_active = false;
    ShowCurrentScreen();
    ui_temp_active = prev;
}

void ShowTempOverlay(const char *line1, const char *line2, uint32_t duration_ms)
{
    ui_temp_active = true;
    LCD_SendLines(line1, line2);
    osTimerStop(uiTempTimerHandle);
    osTimerStart(uiTempTimerHandle, duration_ms);
}

/* ====================== SERVO / DOOR ====================== */
static void MX_TIM3_Init(void)
{
    TIM_OC_InitTypeDef sConfigOC = {0};
    __HAL_RCC_TIM3_CLK_ENABLE();
    htim3.Instance               = TIM3;
    htim3.Init.Prescaler         = 1440 - 1;
    htim3.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim3.Init.Period            = 1000 - 1;
    htim3.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) Error_Handler();
    sConfigOC.OCMode     = TIM_OCMODE_PWM1;
    sConfigOC.Pulse      = SERVO_CCR_90_DEG;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
        Error_Handler();
}

void Servo_SetAngle(uint8_t angle)
{
    uint32_t ccr;
    if (angle > 180U) angle = 180U;
    ccr = SERVO_CCR_0_DEG +
          ((uint32_t)angle * (SERVO_CCR_180_DEG - SERVO_CCR_0_DEG)) / 180U;
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, ccr);
}

void Door_Open(void)  { Servo_SetAngle(DOOR_OPEN_ANGLE);  }
void Door_Close(void) { Servo_SetAngle(DOOR_CLOSE_ANGLE); }

/* ====================== DOOR CONTROL LOGIC ====================== */
void BeginUserWelcome(const char *welcome_line2)
{
    wrong_pass_count = 0U;
    auto_mode_enabled = false;
    ClearInput();
    Door_Open();
    osTimerStop(secondTimerHandle);
    extend_seconds_left     = 0UL;
    door_remaining_seconds  = 0UL;
    pending_user_session    = true;
    CopyLine(pending_welcome_line, welcome_line2);
    ShowTempOverlay("OPEN DOOR", pending_welcome_line, 1500U);
}

void StartUserExtendPhase(void)
{
    extend_seconds_left    = USER_EXTEND_WINDOW_SEC;
    door_remaining_seconds = 0UL;
    app_state              = ST_USER_EXTEND_WAIT;
    osTimerStop(secondTimerHandle);
    osTimerStart(secondTimerHandle, 1000U);
    ShowCurrentScreen();
}

void ReturnToLogin(void)
{
    ClearInput();
    app_state = ST_LOGIN;
    ShowCurrentScreen();
}

void StartUserOpenSession(void)
{
    wrong_pass_count  = 0U;
    auto_mode_enabled = false;
    ClearInput();
    Door_Open();
    extend_seconds_left    = USER_EXTEND_WINDOW_SEC;
    door_remaining_seconds = 0UL;
    app_state              = ST_USER_EXTEND_WAIT;
    osTimerStop(secondTimerHandle);
    osTimerStart(secondTimerHandle, 1000U);
    ShowCurrentScreen();
}

void FinishCloseSequence(void)
{
    pending_user_session     = false;
    ClearInput();
    extend_seconds_left      = 0UL;
    door_remaining_seconds   = 0UL;
    wait_close_retry_seconds = 0UL;
    Door_Close();
    osTimerStop(secondTimerHandle);

    if (close_target_after_wait == CLOSE_TARGET_AUTO_ARMED)
    {
        if (auto_mode_enabled && auto_mode_seconds_left > 0UL)
        {
            app_state = ST_AUTO_ARMED;
            osTimerStart(secondTimerHandle, 1000U);
            ShowCurrentScreen();
            return;
        }
        auto_mode_enabled      = false;
        auto_mode_seconds_left = 0UL;
        ReturnToLogin();
        return;
    }

    if (auto_mode_enabled && auto_mode_seconds_left > 0UL)
    {
        app_state = ST_AUTO_ARMED;
        osTimerStart(secondTimerHandle, 1000U);
        ShowCurrentScreen();
        return;
    }
    auto_mode_enabled      = false;
    auto_mode_seconds_left = 0UL;
    ReturnToLogin();
}

void HandleObstacleDetected(void)
{
    Door_Open();
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);
    osTimerStop(alarmTimerHandle);
    osTimerStart(alarmTimerHandle, 2000U);
    wait_close_retry_seconds = 0UL;
    extend_seconds_left      = USER_EXTEND_WINDOW_SEC;
    door_remaining_seconds   = 0UL;
    app_state                = ST_USER_EXTEND_WAIT;
    osTimerStop(secondTimerHandle);
    osTimerStart(secondTimerHandle, 1000U);
    ShowTempOverlay("Co chuong ngai", "Vat! Mo cua lai", 2000U);
}

void BeginWaitDoorClose(CloseTarget target)
{
    close_target_after_wait  = target;
    pending_user_session     = false;
    ClearInput();
    extend_seconds_left      = 0UL;
    door_remaining_seconds   = 0UL;
    wait_close_retry_seconds = WAIT_CLOSE_RETRY_SEC;
    Door_Close();
    osTimerStop(secondTimerHandle);
    osTimerStart(secondTimerHandle, 1000U);
    app_state = ST_WAIT_DOOR_CLOSE;
    ShowCurrentScreen();
}

void CloseDoorToAutoArmed(void)
{
    pending_user_session = false;
    ClearInput();
    if (DoorSensor_IsClosed())
    {
        close_target_after_wait = CLOSE_TARGET_AUTO_ARMED;
        FinishCloseSequence();
    }
    else BeginWaitDoorClose(CLOSE_TARGET_AUTO_ARMED);
}

void CloseDoorNow(void)
{
    pending_user_session = false;
    ClearInput();
    if (DoorSensor_IsClosed())
    {
        close_target_after_wait = CLOSE_TARGET_NORMAL;
        FinishCloseSequence();
    }
    else BeginWaitDoorClose(CLOSE_TARGET_NORMAL);
}

void EnterAutoMode(void)
{
    auto_mode_enabled      = true;
    auto_mode_seconds_left = 0UL;
    pending_user_session   = false;
    ClearInput();
    extend_seconds_left    = 0UL;
    door_remaining_seconds = 0UL;
    osTimerStop(secondTimerHandle);
    if (DoorSensor_IsClosed())
    {
        Door_Close();
        app_state = ST_AUTO_ARMED;
        ShowCurrentScreen();
    }
    else BeginWaitDoorClose(CLOSE_TARGET_AUTO_ARMED);
}

void ExitAutoModeImmediate(void)
{
    auto_mode_enabled      = false;
    auto_mode_seconds_left = 0UL;
    pending_user_session   = false;
    ClearInput();
    extend_seconds_left    = 0UL;
    door_remaining_seconds = 0UL;
    osTimerStop(secondTimerHandle);
    if (DoorSensor_IsClosed())
    {
        close_target_after_wait = CLOSE_TARGET_NORMAL;
        FinishCloseSequence();
    }
    else BeginWaitDoorClose(CLOSE_TARGET_NORMAL);
}

void StartAutoDetectedSession(void)
{
    wrong_pass_count       = 0U;
    pending_user_session   = false;
    ClearInput();
    Door_Open();
    extend_seconds_left    = USER_EXTEND_WINDOW_SEC;
    door_remaining_seconds = 0UL;
    app_state              = ST_USER_EXTEND_WAIT;
    osTimerStop(secondTimerHandle);
    osTimerStart(secondTimerHandle, 1000U);
    ShowCurrentScreen();
}

void ApplyUserExtension(void)
{
    uint32_t minutes = ParseMinutesSafe(input_buffer);
    if (minutes == 0UL) { CloseDoorNow(); return; }
    door_remaining_seconds = minutes * 60UL;
    ClearInput();
    app_state = ST_USER_OPEN_HOLD;
    ShowCurrentScreen();
}

void ApplyDoorHoldExtension(void)
{
    uint32_t minutes = ParseMinutesSafe(input_buffer);
    if (minutes == 0UL) { ClearInput(); ShowCurrentScreen(); return; }
    door_remaining_seconds += minutes * 60UL;
    if (door_remaining_seconds > (EXTEND_MINUTES_MAX * 60UL))
        door_remaining_seconds = EXTEND_MINUTES_MAX * 60UL;
    ClearInput();
    ShowCurrentScreen();
}

void ApplyAutoModeExtension(void)
{
    uint32_t minutes = ParseAutoModeMinutes(input_buffer);
    if (minutes == 0UL) { ClearInput(); ShowCurrentScreen(); return; }
    auto_mode_seconds_left = minutes * 60UL;
    auto_mode_enabled      = true;
    osTimerStop(secondTimerHandle);
    osTimerStart(secondTimerHandle, 1000U);
    ClearInput();
    ShowTempOverlay("AUTO mode +phut", "A1/A2 da luu", 800U);
}

/* ====================== EVENT POST ====================== */
void PostKeyEvent(char key)
{
    AppEvent evt;
    memset(&evt, 0, sizeof(evt));
    evt.type = EVT_KEY;
    evt.key  = key;
    osMessageQueuePut(appEventQueueHandle, &evt, 0U, 0U);
}

void PostCardEvent(uint8_t *uid)
{
    AppEvent evt;
    memset(&evt, 0, sizeof(evt));
    evt.type = EVT_CARD;
    memcpy(evt.uid, uid, UID_LEN);
    osMessageQueuePut(appEventQueueHandle, &evt, 0U, 0U);
}

void PostSimpleEvent(AppEventType type)
{
    AppEvent evt;
    memset(&evt, 0, sizeof(evt));
    evt.type = type;
    osMessageQueuePut(appEventQueueHandle, &evt, 0U, 0U);
}

/*
 * PostFpEvent: Gửi event vân tay kèm fp_id.
 * Gọi từ FingerTask — không bao giờ gọi trực tiếp từ các task khác.
 */
void PostFpEvent(AppEventType type, uint16_t fp_id)
{
    AppEvent evt;
    memset(&evt, 0, sizeof(evt));
    evt.type  = type;
    evt.fp_id = fp_id;
    osMessageQueuePut(appEventQueueHandle, &evt, 0U, 0U);
}

/* ====================== TIMERS ====================== */
void SecondTimerCallback(void *argument) { (void)argument; PostSimpleEvent(EVT_SECOND_TICK);     }
void UiTempTimerCallback(void *argument) { (void)argument; PostSimpleEvent(EVT_UI_TEMP_TIMEOUT); }
void AlarmTimerCallback(void *argument)  { (void)argument; PostSimpleEvent(EVT_ALARM_TIMEOUT);   }

/* ====================== KEYPAD ====================== */
char Keypad_ScanRaw(void)
{
    const char    keymap[4][4]  = {
        {'1','2','3','A'}, {'4','5','6','B'},
        {'7','8','9','C'}, {'*','0','#','D'}
    };
    GPIO_TypeDef *colPorts[4]   = {GPIOA, GPIOB, GPIOB, GPIOB};
    uint16_t      colPins[4]    = {GPIO_PIN_8, GPIO_PIN_15, GPIO_PIN_14, GPIO_PIN_13};
    GPIO_TypeDef *rowPorts[4]   = {GPIOA, GPIOA, GPIOA, GPIOA};
    uint16_t      rowPins[4]    = {GPIO_PIN_12, GPIO_PIN_11, GPIO_PIN_10, GPIO_PIN_9};
    volatile uint32_t settle;
    int c, r;

    for (c = 0; c < 4; c++)
    {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15|GPIO_PIN_14|GPIO_PIN_13, GPIO_PIN_SET);
        HAL_GPIO_WritePin(colPorts[c], colPins[c], GPIO_PIN_RESET);
        for (settle = 0; settle < 200U; settle++) { __NOP(); }
        for (r = 0; r < 4; r++)
            if (HAL_GPIO_ReadPin(rowPorts[r], rowPins[r]) == GPIO_PIN_RESET)
                return keymap[r][c];
    }
    return 0;
}

/* ==============================================================
 * END OF PART 1
 * Tiếp theo: HandleKeyEvent, HandleCardEvent, HandleAppEvent,
 *            Tasks, main(), peripheral inits
 * ============================================================== */
/* ==============================================================
 * main.c  (PART 2 / 2)
 * HandleKeyEvent -> HandleCardEvent -> HandleAppEvent
 * -> Tasks -> main() -> Peripheral inits
 * ============================================================== */

/* ====================== APP HANDLERS ====================== */

void HandleKeyEvent(char key)
{
    switch (app_state)
    {
        case ST_LOGIN:
            if (key >= '0' && key <= '9') { AppendInput(key); ShowCurrentScreen(); }
            else if (key == '*') { BackspaceInput(); ShowCurrentScreen(); }
            else if (key == '#')
            {
                if (!IsValidPasswordLength(input_len))
                    { ReturnToLogin(); ShowTempOverlay("MK 2-16 so", "", 1000U); }
                else if (strcmp(input_buffer, admin_password) == 0)
                    { wrong_pass_count = 0U; ClearInput(); app_state = ST_ADMIN_MENU; ShowCurrentScreen(); }
                else if (strcmp(input_buffer, user_password) == 0)
                    { BeginUserWelcome("Chao mung user"); }
                else
                {
                    wrong_pass_count++;
                    ReturnToLogin();
                    if (wrong_pass_count >= 3U)
                    {
                        wrong_pass_count = 0U;
                        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);
                        osTimerStop(alarmTimerHandle);
                        osTimerStart(alarmTimerHandle, 2000U);
                        ShowTempOverlay("Sai 3 lan", "Canh bao", 2000U);
                    }
                    else ShowTempOverlay("Sai mat khau", "", 1000U);
                }
            }
            break;

        case ST_ADMIN_MENU:
            if      (key == '1') { app_state = ST_ADMIN_ADD_MENU;    ShowCurrentScreen(); }
            else if (key == '2') { app_state = ST_ADMIN_DELETE_MENU; ShowCurrentScreen(); }
            else if (key == '3') { app_state = ST_CHANGE_PW_MENU;    ShowCurrentScreen(); }
            else if (key == '4') { app_state = ST_ADMIN_MENU_2;      ShowCurrentScreen(); }
            break;

        case ST_ADMIN_MENU_2:
            if      (key == '4') { app_state = ST_ADMIN_MENU; ShowCurrentScreen(); }
            else if (key == '5') { EnterAutoMode(); }
            else if (key == '6') { ReturnToLogin(); }
            break;

        case ST_ADMIN_ADD_MENU:
            if      (key == '1') { app_state = ST_ADMIN_ADD_CARD_WAIT; ShowCurrentScreen(); }
            else if (key == '2') { app_state = ST_ADMIN_ADD_FP_WAIT;   ShowCurrentScreen(); }
            else if (key == '3') { app_state = ST_ADMIN_MENU;          ShowCurrentScreen(); }
            break;

        case ST_ADMIN_DELETE_MENU:
            if      (key == '1') { app_state = ST_ADMIN_DELETE_CARD_WAIT; ShowCurrentScreen(); }
            else if (key == '2') { app_state = ST_ADMIN_DELETE_FP_WAIT;   ShowCurrentScreen(); }
            else if (key == '3') { app_state = ST_ADMIN_MENU;             ShowCurrentScreen(); }
            break;

        case ST_ADMIN_ADD_CARD_WAIT:
            if (key == '*') { app_state = ST_ADMIN_ADD_MENU; ShowCurrentScreen(); }
            break;

        case ST_ADMIN_DELETE_CARD_WAIT:
            if (key == '*') { app_state = ST_ADMIN_DELETE_MENU; ShowCurrentScreen(); }
            break;

        /* Nhan * de thoat khoi fingerprint enroll/delete */
        case ST_ADMIN_ADD_FP_WAIT:
            if (key == '*') { app_state = ST_ADMIN_ADD_MENU; ShowCurrentScreen(); }
            break;

        case ST_ADMIN_DELETE_FP_WAIT:
            if (key == '*') { app_state = ST_ADMIN_DELETE_MENU; ShowCurrentScreen(); }
            break;

        case ST_CHANGE_PW_MENU:
            if      (key == '1') { ClearInput(); app_state = ST_CHANGE_ADMIN_OLD; ShowCurrentScreen(); }
            else if (key == '2') { ClearInput(); app_state = ST_CHANGE_USER_OLD;  ShowCurrentScreen(); }
            else if (key == '3') { app_state = ST_ADMIN_MENU; ShowCurrentScreen(); }
            break;

        case ST_CHANGE_ADMIN_OLD:
            if (key >= '0' && key <= '9') { AppendInput(key); ShowCurrentScreen(); }
            else if (key == '*') { BackspaceInput(); ShowCurrentScreen(); }
            else if (key == '#')
            {
                if (strcmp(input_buffer, admin_password) == 0)
                    { ClearInput(); app_state = ST_CHANGE_ADMIN_NEW; ShowCurrentScreen(); }
                else
                    { ClearInput(); app_state = ST_CHANGE_PW_MENU; ShowCurrentScreen();
                      ShowTempOverlay("Sai MK cu", "", 1000U); }
            }
            break;

        case ST_CHANGE_ADMIN_NEW:
            if (key >= '0' && key <= '9') { AppendInput(key); ShowCurrentScreen(); }
            else if (key == '*') { BackspaceInput(); ShowCurrentScreen(); }
            else if (key == '#')
            {
                if (IsValidPasswordLength(input_len))
                    { strcpy(admin_password, input_buffer); ClearInput();
                      app_state = ST_CHANGE_PW_MENU; ShowCurrentScreen();
                      ShowTempOverlay("Doi MK Admin OK", "", 1000U); }
                else
                    { ClearInput(); ShowCurrentScreen();
                      ShowTempOverlay("MK 2-16 so", "", 1000U); }
            }
            break;

        case ST_CHANGE_USER_OLD:
            if (key >= '0' && key <= '9') { AppendInput(key); ShowCurrentScreen(); }
            else if (key == '*') { BackspaceInput(); ShowCurrentScreen(); }
            else if (key == '#')
            {
                if (strcmp(input_buffer, user_password) == 0)
                    { ClearInput(); app_state = ST_CHANGE_USER_NEW; ShowCurrentScreen(); }
                else
                    { ClearInput(); app_state = ST_CHANGE_PW_MENU; ShowCurrentScreen();
                      ShowTempOverlay("Sai MK cu", "", 1000U); }
            }
            break;

        case ST_CHANGE_USER_NEW:
            if (key >= '0' && key <= '9') { AppendInput(key); ShowCurrentScreen(); }
            else if (key == '*') { BackspaceInput(); ShowCurrentScreen(); }
            else if (key == '#')
            {
                if (IsValidPasswordLength(input_len))
                    { strcpy(user_password, input_buffer); ClearInput();
                      app_state = ST_CHANGE_PW_MENU; ShowCurrentScreen();
                      ShowTempOverlay("Doi MK User OK", "", 1000U); }
                else
                    { ClearInput(); ShowCurrentScreen();
                      ShowTempOverlay("MK 2-16 so", "", 1000U); }
            }
            break;

        case ST_AUTO_ARMED:
            if      (key == 'C') ExitAutoModeImmediate();
            else if (key == 'D') CloseDoorToAutoArmed();
            else if ((key == 'A' || key == 'a') && input_len == 0U)
                { AppendInput('A'); ShowCurrentScreen(); }
            else if (key >= '0' && key <= '9' && input_len > 0U &&
                     (input_buffer[0] == 'A' || input_buffer[0] == 'a'))
                { AppendInput(key); ShowCurrentScreen(); }
            else if (key == '*') { BackspaceInput(); ShowCurrentScreen(); }
            else if (key == '#' && InputIsAutoModeMinutes()) ApplyAutoModeExtension();
            break;

        case ST_USER_EXTEND_WAIT:
            if ((key == 'A' || key == 'a') && auto_mode_enabled && input_len == 0U)
                { AppendInput('A'); ShowCurrentScreen(); }
            else if (key >= '0' && key <= '9') { AppendInput(key); ShowCurrentScreen(); }
            else if (key == '*') { BackspaceInput(); ShowCurrentScreen(); }
            else if (key == '#')
            {
                if (auto_mode_enabled && InputIsAutoModeMinutes()) ApplyAutoModeExtension();
                else ApplyUserExtension();
            }
            else if (key == 'D')
                { if (auto_mode_enabled) CloseDoorToAutoArmed(); else CloseDoorNow(); }
            else if (key == 'C')
                { if (auto_mode_enabled) ExitAutoModeImmediate(); }
            break;

        case ST_USER_OPEN_HOLD:
            if ((key == 'A' || key == 'a') && auto_mode_enabled && input_len == 0U)
                { AppendInput('A'); ShowCurrentScreen(); }
            else if (key >= '0' && key <= '9') { AppendInput(key); ShowCurrentScreen(); }
            else if (key == '*') { BackspaceInput(); ShowCurrentScreen(); }
            else if (key == '#')
            {
                if (auto_mode_enabled && InputIsAutoModeMinutes()) ApplyAutoModeExtension();
                else if (input_len > 0U) ApplyDoorHoldExtension();
            }
            else if (key == 'D')
                { if (auto_mode_enabled) CloseDoorToAutoArmed(); else CloseDoorNow(); }
            else if (key == 'C')
                { if (auto_mode_enabled) ExitAutoModeImmediate(); }
            break;

        case ST_WAIT_DOOR_CLOSE:
            if (key == 'C' && auto_mode_enabled) ExitAutoModeImmediate();
            break;

        default:
            break;
    }
}

/* ---- CARD EVENT ---- */
void HandleCardEvent(uint8_t *uid)
{
    int  slot;
    char msg[LCD_LINE_LEN + 1];

    switch (app_state)
    {
        case ST_LOGIN:
            slot = FindCard(uid);
            if (slot >= 0)
                { BuildUserMessage(msg, "Chao mung", slot); BeginUserWelcome(msg); }
            else
                ShowTempOverlay("The khong dung", "", 800U);
            break;

        case ST_ADMIN_ADD_CARD_WAIT:
            if (FindCard(uid) >= 0)
            {
                app_state = ST_ADMIN_ADD_MENU; ShowCurrentScreen();
                ShowTempOverlay("The da ton tai", "", 1000U);
                return;
            }
            slot = FindEmptyCardSlot();
            if (slot < 0)
            {
                app_state = ST_ADMIN_ADD_MENU; ShowCurrentScreen();
                ShowTempOverlay("Danh sach day", "", 1000U);
                return;
            }
            memcpy(registered_cards[slot], uid, UID_LEN);
            card_valid[slot] = 1U;
            Card_Flash_Save();

            app_state = ST_ADMIN_ADD_MENU;
            ShowCurrentScreen();
            BuildUserMessage(msg, "Da them", slot);
            ShowTempOverlay(msg, "", 1000U);
            break;

        case ST_ADMIN_DELETE_CARD_WAIT:
            slot = FindCard(uid);
            if (slot < 0)
            {
                app_state = ST_ADMIN_DELETE_MENU; ShowCurrentScreen();
                ShowTempOverlay("Khong tim thay", "", 1000U);
                return;
            }
            memset(registered_cards[slot], 0, UID_LEN);
            card_valid[slot] = 0U;
            Card_Flash_Save();

            app_state = ST_ADMIN_DELETE_MENU;
            ShowCurrentScreen();
            BuildUserMessage(msg, "Da xoa", slot);
            ShowTempOverlay(msg, "", 1000U);
            break;

        default:
            break;
    }
}

/* ====================== MAIN APP EVENT HANDLER ====================== */
void HandleAppEvent(const AppEvent *evt)
{
    char buf[LCD_LINE_LEN + 1];

    if (evt == NULL) return;

    if (ui_temp_active)
    {
    	 if (evt->type == EVT_KEY && evt->key == '*' &&
    	            (app_state == ST_ADMIN_ADD_FP_WAIT ||
    	             app_state == ST_ADMIN_DELETE_FP_WAIT))
    	        {
    	            ui_temp_active = false;
    	            osTimerStop(uiTempTimerHandle);
    	            HandleKeyEvent(evt->key);
    	            return;
    	        }

    	        if (evt->type != EVT_UI_TEMP_TIMEOUT &&
    	            evt->type != EVT_ALARM_TIMEOUT   &&
    	            evt->type != EVT_SECOND_TICK     &&
    	            evt->type != EVT_DOOR_CLOSED)
    	            return;
    }

    switch (evt->type)
    {
        case EVT_KEY:
            HandleKeyEvent(evt->key);
            break;

        case EVT_CARD:
            HandleCardEvent((uint8_t *)evt->uid);
            break;

        case EVT_SECOND_TICK:
            if (auto_mode_enabled && auto_mode_seconds_left > 0UL)
                auto_mode_seconds_left--;

            if (app_state == ST_USER_EXTEND_WAIT)
            {
                if (extend_seconds_left > 0UL) extend_seconds_left--;
                if (extend_seconds_left == 0UL)
                {
                    if (input_len > 0U)
                    {
                        if (auto_mode_enabled && InputIsAutoModeMinutes())
                            { ApplyAutoModeExtension(); CloseDoorNow(); }
                        else ApplyUserExtension();
                    }
                    else CloseDoorNow();
                }
                else ShowCurrentScreen();
            }
            else if (app_state == ST_USER_OPEN_HOLD)
            {
                if (door_remaining_seconds > 0UL) door_remaining_seconds--;
                if (door_remaining_seconds == 0UL) CloseDoorNow();
                else ShowCurrentScreen();
            }
            else if (app_state == ST_AUTO_ARMED)
            {
                if (auto_mode_enabled && auto_mode_seconds_left == 0UL)
                    ExitAutoModeImmediate();
                else ShowCurrentScreen();
            }
            else if (app_state == ST_WAIT_DOOR_CLOSE)
            {
                if (DoorSensor_IsClosed())
                {
                    FinishCloseSequence();
                }
                else if (wait_close_retry_seconds > 0UL)
                {
                    wait_close_retry_seconds--;
                    if (wait_close_retry_seconds == 0UL)
                        HandleObstacleDetected();
                    else
                        ShowCurrentScreen();
                }
            }
            break;

        case EVT_UI_TEMP_TIMEOUT:
            ui_temp_active = false;
            if (pending_user_session)
            {
                pending_user_session = false;
                StartUserExtendPhase();
            }
            else ForceShowCurrentScreen();
            break;

        case EVT_ALARM_TIMEOUT:
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET);
            break;

        case EVT_AUTO_NEAR:
            if (auto_mode_enabled && app_state == ST_AUTO_ARMED)
                StartAutoDetectedSession();
            break;

        case EVT_DOOR_CLOSED:
            if (app_state == ST_WAIT_DOOR_CLOSE && DoorSensor_IsClosed())
                FinishCloseSequence();
            break;

        /* ====================================================
         * FINGERPRINT EVENTS (post tu FingerTask)
         * ==================================================== */

        case EVT_FP_MATCH:
            /* Chi xu ly o ST_LOGIN, cac state khac bo qua */
            if (app_state == ST_LOGIN)
            {
                snprintf(buf, sizeof(buf), "FP ID:%u OK", (unsigned)evt->fp_id);
                BeginUserWelcome(buf);
            }
            break;

        case EVT_FP_NOTFOUND:
            if (app_state == ST_LOGIN)
                ShowTempOverlay("Van tay sai", "", 800U);
            else if (app_state == ST_ADMIN_DELETE_FP_WAIT)
                ShowTempOverlay("Khong tim thay", "Thu lai", 1000U);
            break;

        case EVT_FP_ERROR:
            if (app_state == ST_LOGIN)
                ShowTempOverlay("Loi scan FP", "", 800U);
            break;

        case EVT_FP_ENROLLED:
            if (app_state == ST_ADMIN_ADD_FP_WAIT)
            {
                snprintf(buf, sizeof(buf), "Luu FP ID:%u", (unsigned)evt->fp_id);
                app_state = ST_ADMIN_ADD_MENU;
                ShowCurrentScreen();
                ShowTempOverlay(buf, "Thanh cong", 1500U);
            }
            break;
        case EVT_FP_ENROLL_FAIL:
            if (app_state == ST_ADMIN_ADD_FP_WAIT)
            {
                if (evt->fp_id == 0xE1U)
                {
                    ShowTempOverlay("Bo nho day", "Khong con ID", 1500U);
                }
                else
                {
                    snprintf(buf, sizeof(buf), "Err:0x%02X", (unsigned)evt->fp_id);
                    ShowTempOverlay("Enroll loi", buf, 1500U);
                }
            }
            break;
        case EVT_FP_DELETED:
            if (app_state == ST_ADMIN_DELETE_FP_WAIT)
            {
                snprintf(buf, sizeof(buf), "Xoa FP ID:%u", (unsigned)evt->fp_id);
                app_state = ST_ADMIN_DELETE_MENU;
                ShowCurrentScreen();
                ShowTempOverlay(buf, "Thanh cong", 1500U);
            }
            break;

        default:
            break;
    }
}

/* ====================== TASKS ====================== */

void StartLCDTask(void *argument)
{
    LcdMessage msg;
    (void)argument;
    for (;;)
    {
        if (osMessageQueueGet(lcdQueueHandle, &msg, NULL, osWaitForever) == osOK)
        {
            LCD_WritePaddedLine(0U, msg.line1);
            LCD_WritePaddedLine(1U, msg.line2);
        }
    }
}

void StartAccessControlTask(void *argument)
{
    AppEvent evt;
    (void)argument;
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET);
    LCD_SendLines("Access task OK", "Dang khoi dong");
    osDelay(500U);
    ReturnToLogin();
    for (;;)
    {
        if (osMessageQueueGet(appEventQueueHandle, &evt, NULL, osWaitForever) == osOK)
            HandleAppEvent(&evt);
    }
}

void StartKeypadTask(void *argument)
{
    char    raw_key, last_raw = 0, stable_key = 0;
    uint8_t stable_count = 0U;
    (void)argument;
    for (;;)
    {
        raw_key = Keypad_ScanRaw();
        if (raw_key == last_raw) { if (stable_count < 3U) stable_count++; }
        else { stable_count = 0U; last_raw = raw_key; }
        if (stable_count >= 2U && stable_key != raw_key)
        {
            stable_key = raw_key;
            if (stable_key != 0) PostKeyEvent(stable_key);
        }
        osDelay(KEYPAD_SCAN_PERIOD_MS);
    }
}

void StartRFIDTask(void *argument)
{
    bool    card_latched = false;
    uint8_t lost_count   = 0U;
    (void)argument;
    for (;;)
    {
        status = MFRC522_Request(PICC_REQIDL, str);
        if (status == MI_OK)
        {
            status = MFRC522_Anticoll(str);
            if (status == MI_OK)
            {
                memcpy(sNum, str, 5);
                lost_count = 0U;
                if (!card_latched) { card_latched = true; PostCardEvent(str); }
            }
        }
        else
        {
            if (lost_count < 3U) lost_count++;
            else { lost_count = 0U; card_latched = false; }
        }
        osDelay(RFID_SCAN_PERIOD_MS);
    }
}

void StartAutoTask(void *argument)
{
    bool near_latched = false;
    (void)argument;
    for (;;)
    {
        if (auto_mode_enabled && app_state == ST_AUTO_ARMED && !ui_temp_active)
        {
            HCSR04_Trigger();
            osDelay(AUTO_ECHO_WAIT_MS);
            if (distance_ready &&
                distance_x10_cm > 0UL &&
                distance_x10_cm <= AUTO_TRIGGER_CM_X10)
            {
                if (!near_latched)
                    { near_latched = true; PostSimpleEvent(EVT_AUTO_NEAR); }
            }
            else near_latched = false;
            osDelay(AUTO_SCAN_PERIOD_MS);
        }
        else { near_latched = false; osDelay(120U); }
    }
}

/*
 * StartFingerTask
 *
 * Nguyen tac:
 *   - Task duy nhat duoc goi fingerprint API.
 *   - Snapshot app_state moi vong (doc 1 lan), khong lock.
 *   - Giao tiep module qua fp_txrx() co fpMutexHandle ben trong.
 *   - Ket qua post vao appEventQueueHandle bang PostFpEvent().
 *   - KHONG goi LCD, KHONG thay doi app_state truc tiep.
 *   - FP_DELAY = vTaskDelay trong fingerprint.c khi co USE_FREERTOS.
 */
/*
 * fp_do_enroll_step: Thực hiện enroll với LCD feedback từng bước.
 * Gọi từ StartFingerTask — chạy trong FingerTask context.
 * Trả về sớm (return) nếu user nhấn * thoát (app_state đổi).
 */
static void fp_do_enroll_step(void)
{
    uint8_t  img, res;
    uint32_t t0;
    uint8_t  removeCnt;
    /* ── Lay ID moi do STM32 quan ly ── */
    uint16_t new_id = FP_AllocateEmptyId();
    if (new_id == 0U)
    {
        PostFpEvent(EVT_FP_ENROLL_FAIL, 0xE1U);
        osDelay(1500U);
        return;
    }
    /* ════════════════════════════════
     * BƯỚC 1: Chờ đặt ngón lần 1
     * ════════════════════════════════ */
    LCD_SendLines("Dat ngon lan 1", "*:Thoat");

    while (1)
    {
        if (app_state != ST_ADMIN_ADD_FP_WAIT) return;
        img = fp_get_image();
        if (img != FINGERPRINT_NOFINGER) break;
        osDelay(80U);
    }
    if (img != FINGERPRINT_OK)
    {
        PostFpEvent(EVT_FP_ENROLL_FAIL, (uint16_t)img);
        osDelay(2500U);
        return;
    }

    /* IMAGE2TZ(1) — ngay lập tức, không delay */
    res = fp_image2tz(1);
    if (res != FINGERPRINT_OK)
    {
        PostFpEvent(EVT_FP_ENROLL_FAIL, (uint16_t)res);
        osDelay(2500U);
        return;
    }
    FP_DELAY(300U);

    /* ════════════════════════════════
     * BƯỚC 2: Chờ nhấc ngón ra
     * ════════════════════════════════ */
    LCD_SendLines("Nhat ngon ra!", "");

    t0        = HAL_GetTick();
    removeCnt = 0U;
    while ((HAL_GetTick() - t0) < 8000U)
    {
        if (app_state != ST_ADMIN_ADD_FP_WAIT) return;
        img = fp_get_image();
        /* AS608 clone: NOFINGER hoặc bất kỳ non-OK đều = không có ngón */
        if (img != FINGERPRINT_OK)
        {
            removeCnt++;
            if (removeCnt >= 3U) break;
        }
        else
        {
            removeCnt = 0U;
        }
        osDelay(80U);
    }

    if (removeCnt < 3U)
    {
        PostFpEvent(EVT_FP_ENROLL_FAIL, 0x32U); /* Timeout nhấc ngón */
        osDelay(2500U);
        return;
    }
    FP_DELAY(400U);  /* Cho module reset image buffer */

    /* ════════════════════════════════
     * BƯỚC 3: Chờ đặt ngón lần 2
     * ════════════════════════════════ */
    LCD_SendLines("Dat ngon lan 2", "*:Thoat");

    while (1)
    {
        if (app_state != ST_ADMIN_ADD_FP_WAIT) return;
        img = fp_get_image();
        if (img != FINGERPRINT_NOFINGER) break;
        osDelay(80U);
    }
    if (img != FINGERPRINT_OK)
    {
        PostFpEvent(EVT_FP_ENROLL_FAIL, (uint16_t)img);
        osDelay(2500U);
        return;
    }

    /* IMAGE2TZ(2) — ngay lập tức */
    res = fp_image2tz(2);
    if (res != FINGERPRINT_OK)
    {
        PostFpEvent(EVT_FP_ENROLL_FAIL, (uint16_t)res);
        osDelay(2500U);
        return;
    }
    FP_DELAY(300U);

    /* ════════════════════════════════
     * BƯỚC 4: Tạo model + Lưu
     * ════════════════════════════════ */
    LCD_SendLines("Dang luu...", "");

    res = fp_create_model();
    if (res != FINGERPRINT_OK)
    {
        PostFpEvent(EVT_FP_ENROLL_FAIL, (uint16_t)res);
        osDelay(2500U);
        return;
    }

    FP_DELAY(200U);
    res = fp_store_model(new_id);

    if (app_state != ST_ADMIN_ADD_FP_WAIT) return;

    if (res == FINGERPRINT_OK)
    {
        FP_MarkIdUsed(new_id);
        FP_Flash_SaveMap();
        PostFpEvent(EVT_FP_ENROLLED, new_id);
    }
    else
    {
        PostFpEvent(EVT_FP_ENROLL_FAIL, (uint16_t)res);
        osDelay(2500U);
    }
}
void StartFingerTask(void *argument)
{
    (void)argument;
    fp_uart_start_it();
    osDelay(100U);
    init_fingerprint();

    if (!FP_Flash_LoadMap())
    {
        FP_IdMap_Clear();
        FP_Flash_SaveMap();
    }
    for (;;)
    {
        AppState cur = app_state;

        /* ── VERIFY ── */
        if (cur == ST_LOGIN         ||
            cur == ST_AUTO_ARMED    ||
            cur == ST_USER_EXTEND_WAIT ||
            cur == ST_USER_OPEN_HOLD)
        {
            uint8_t img = fp_get_image();

            if (img == FINGERPRINT_NOFINGER)
            {
                /* Không có ngón — im lặng, không post event */
                osDelay(150U);
            }
            else if (img == FINGERPRINT_OK)
            {
                uint8_t tz = fp_image2tz(2);

                if (tz != FINGERPRINT_OK)
                {
                    /*
                     * Phân biệt lỗi chất lượng ảnh vs lỗi giao tiếp:
                     * IMAGEMESS / FEATUREFAIL / INVALIDIMAGE = ảnh kém → "Van tay sai"
                     * Còn lại (0x01 = UART lỗi, ...) = lỗi thật → "Loi scan FP"
                     */
                    if (tz == FINGERPRINT_IMAGEMESS   ||
                        tz == FINGERPRINT_FEATUREFAIL ||
                        tz == FINGERPRINT_INVALIDIMAGE)
                    {
                        PostFpEvent(EVT_FP_NOTFOUND, 0U);   /* hiện "Van tay sai" */
                    }
                    else
                    {
                        PostFpEvent(EVT_FP_ERROR, 0U);      /* hiện "Loi scan FP" */
                    }
                    fp_wait_finger_removed_timeout(2000U);
                    osDelay(150U);
                }
                else
                {
                    /* Ảnh tốt → search database */
                    uint8_t sdata[6] = {
                        FINGERPRINT_SEARCH, 0x02U,
                        0x00U, 0x00U,
                        (uint8_t)(capacity >> 8),
                        (uint8_t)(capacity & 0xFFU)
                    };
                    setup_packet(sdata, 6);
                    uint8_t res = fp_txrx();

                    if (res == FINGERPRINT_OK)
                    {
                        uint16_t id = ((uint16_t)rpacket.data[1] << 8)
                                      | (uint16_t)rpacket.data[2];
                        PostFpEvent(EVT_FP_MATCH, id);
                    }
                    else if (res == FINGERPRINT_NOTFOUND)
                    {
                        PostFpEvent(EVT_FP_NOTFOUND, 0U);   /* hiện "Van tay sai" */
                    }
                    else
                    {
                        PostFpEvent(EVT_FP_ERROR, 0U);      /* lỗi thật */
                    }

                    fp_wait_finger_removed_timeout(3000U);
                    osDelay(200U);
                }
            }
            else
            {
                /* Lỗi module khác — đợi ngắn rồi thử lại */
                osDelay(200U);
            }
        }

        /* ── ENROLL ── */
        else if (cur == ST_ADMIN_ADD_FP_WAIT)
        {
            fp_do_enroll_step();   /* Toàn bộ logic đã chuyển vào đây */
            osDelay(300U);
        }

        /* ── DELETE ── */
        else if (cur == ST_ADMIN_DELETE_FP_WAIT)
        {
            uint8_t img = fp_get_image();

            if (img == FINGERPRINT_NOFINGER)
            {
                osDelay(150U);
            }
            else if (img == FINGERPRINT_OK)
            {
                uint8_t tz = fp_image2tz(2);
                if (tz == FINGERPRINT_OK)
                {
                    uint8_t sdata[6] = {
                        FINGERPRINT_SEARCH, 0x02U,
                        0x00U, 0x00U,
                        (uint8_t)(capacity >> 8),
                        (uint8_t)(capacity & 0xFFU)
                    };
                    setup_packet(sdata, 6);
                    uint8_t res = fp_txrx();

                    if (res == FINGERPRINT_OK)
                    {
                        uint16_t id = ((uint16_t)rpacket.data[1] << 8)
                                      | (uint16_t)rpacket.data[2];
                        uint8_t del = fp_delete_model(id);
                        if (del == FINGERPRINT_OK)
                        {
                            FP_ReleaseId(id);
                            FP_Flash_SaveMap();
                            PostFpEvent(EVT_FP_DELETED, id);
                        }
                        else
                        {
                            PostFpEvent(EVT_FP_ENROLL_FAIL, (uint16_t)del);
                        }
                    }
                    else PostFpEvent(EVT_FP_NOTFOUND, 0U);
                }
                fp_wait_finger_removed_timeout(3000U);
                osDelay(200U);
            }
            else osDelay(200U);
        }

        else { osDelay(200U); }
    }
}
/* ====================== MAIN ====================== */
int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_SPI1_Init();
    MX_USART3_UART_Init();
    MX_TIM2_Init();
    MX_TIM3_Init();

    CLCD_I2C_Init(&LCD1, &hi2c1, 0x4E, 16, 2);
    MFRC522_Init();
    if (!Card_Flash_Load())
    {
        Card_Flash_Save();   /* lần đầu: lưu bộ 3 thẻ mặc định xuống Flash */
    }

    HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, SERVO_CCR_90_DEG);
    HAL_Delay(500);
    Door_Close();
    HAL_Delay(300);

    CLCD_I2C_Clear(&LCD1);
    LCD_WritePaddedLine(0U, "RTOS Boot...");
    LCD_WritePaddedLine(1U, "Init objects");

    osKernelInitialize();

    lcdMutexHandle = osMutexNew(NULL);
    fpMutexHandle  = osMutexNew(NULL);

    appEventQueueHandle = osMessageQueueNew(12U, sizeof(AppEvent),   NULL);
    lcdQueueHandle      = osMessageQueueNew(4U,  sizeof(LcdMessage), NULL);
    secondTimerHandle   = osTimerNew(SecondTimerCallback, osTimerPeriodic, NULL, NULL);
    uiTempTimerHandle   = osTimerNew(UiTempTimerCallback, osTimerOnce,     NULL, NULL);
    alarmTimerHandle    = osTimerNew(AlarmTimerCallback,  osTimerOnce,     NULL, NULL);

    if (lcdMutexHandle == NULL || fpMutexHandle == NULL ||
        appEventQueueHandle == NULL || lcdQueueHandle == NULL ||
        secondTimerHandle == NULL || uiTempTimerHandle == NULL ||
        alarmTimerHandle == NULL)
    {
        LCD_WritePaddedLine(0U, "RTOS obj fail");
        LCD_WritePaddedLine(1U, "Heap/Mutex err");
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);
        while (1) {}
    }

    const osThreadAttr_t keypadTask_attr  = { .name = "KeypadTask",  .priority = osPriorityBelowNormal, .stack_size = 256U  };
    const osThreadAttr_t rfidTask_attr    = { .name = "RFIDTask",    .priority = osPriorityBelowNormal, .stack_size = 384U  };
    const osThreadAttr_t accessTask_attr  = { .name = "AccessTask",  .priority = osPriorityAboveNormal, .stack_size = 1024U };
    const osThreadAttr_t lcdTask_attr     = { .name = "LCDTask",     .priority = osPriorityNormal,      .stack_size = 640U  };
    const osThreadAttr_t autoTask_attr    = { .name = "AutoTask",    .priority = osPriorityBelowNormal, .stack_size = 384U  };
    const osThreadAttr_t fingerTask_attr  = {
        /* 768B: save_fingerprint dung buffer stack + overhead RTOS.
           Tang len 1024U neu gap stack overflow. */
        .name = "FingerTask", .priority = osPriorityBelowNormal, .stack_size = 2048U
    };

    keypadTaskHandle = osThreadNew(StartKeypadTask,        NULL, &keypadTask_attr);
    rfidTaskHandle   = osThreadNew(StartRFIDTask,          NULL, &rfidTask_attr);
    accessTaskHandle = osThreadNew(StartAccessControlTask, NULL, &accessTask_attr);
    lcdTaskHandle    = osThreadNew(StartLCDTask,           NULL, &lcdTask_attr);
    autoTaskHandle   = osThreadNew(StartAutoTask,          NULL, &autoTask_attr);
    fingerTaskHandle = osThreadNew(StartFingerTask,        NULL, &fingerTask_attr);

    if (keypadTaskHandle == NULL || rfidTaskHandle   == NULL ||
        accessTaskHandle == NULL || lcdTaskHandle    == NULL ||
        autoTaskHandle   == NULL || fingerTaskHandle == NULL)
    {
        LCD_WritePaddedLine(0U, "Task fail");
        LCD_WritePaddedLine(1U, "Check heap");
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);
        while (1) {}
    }

    osKernelStart();

    LCD_WritePaddedLine(0U, "Kernel err");
    LCD_WritePaddedLine(1U, "Check RTOS cfg");
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);
    while (1) {}
}

/* ====================== PERIPHERAL INIT ====================== */
static void MX_TIM2_Init(void)
{
    TIM_ClockConfigTypeDef  sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig      = {0};
    TIM_IC_InitTypeDef      sConfigIC          = {0};
    __HAL_RCC_TIM2_CLK_ENABLE();
    htim2.Instance = TIM2; htim2.Init.Prescaler = 72-1;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP; htim2.Init.Period = 65535;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim2) != HAL_OK) Error_Handler();
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) Error_Handler();
    if (HAL_TIM_IC_Init(&htim2) != HAL_OK) Error_Handler();
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) Error_Handler();
    sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
    sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
    sConfigIC.ICPrescaler = TIM_ICPSC_DIV1; sConfigIC.ICFilter = 0;
    if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK) Error_Handler();
    HAL_NVIC_SetPriority(TIM2_IRQn, 5, 0); HAL_NVIC_EnableIRQ(TIM2_IRQn);
}

static void MX_I2C1_Init(void)
{
    hi2c1.Instance = I2C1; hi2c1.Init.ClockSpeed = 100000;
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2; hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE; hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK) Error_Handler();
}
static void MX_USART3_UART_Init(void)
{
    huart3.Instance          = USART3;
    huart3.Init.BaudRate     = 57600;
    huart3.Init.WordLength   = UART_WORDLENGTH_8B;
    huart3.Init.StopBits     = UART_STOPBITS_1;
    huart3.Init.Parity       = UART_PARITY_NONE;
    huart3.Init.Mode         = UART_MODE_TX_RX;
    huart3.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart3.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart3) != HAL_OK) Error_Handler();

    /* THÊM: bật NVIC cho USART3 — priority 5 tương thích FreeRTOS */
    HAL_NVIC_SetPriority(USART3_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART3_IRQn);
}
static void MX_SPI1_Init(void)
{
    hspi1.Instance = SPI1; hspi1.Init.Mode = SPI_MODE_MASTER;
    hspi1.Init.Direction = SPI_DIRECTION_2LINES; hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity = SPI_POLARITY_LOW; hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi1.Init.NSS = SPI_NSS_SOFT; hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
    hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB; hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE; hspi1.Init.CRCPolynomial = 10;
    if (HAL_SPI_Init(&hspi1) != HAL_OK) Error_Handler();
}



static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOA_CLK_ENABLE(); __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE(); __HAL_RCC_AFIO_CLK_ENABLE();

    /* PC14 Alarm LED */
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin = GPIO_PIN_14; GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW; HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* HC-SR04 Trig PA1 */
    HAL_GPIO_WritePin(GPIOA, HCSR04_TRIG_Pin, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin = HCSR04_TRIG_Pin; GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW; HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* HC-SR04 Echo PA0 */
    GPIO_InitStruct.Pin = GPIO_PIN_0; GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL; HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* RC522 CS PA4 */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    GPIO_InitStruct.Pin = GPIO_PIN_4; GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH; HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* RC522 RST PB0 */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
    GPIO_InitStruct.Pin = GPIO_PIN_0; GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH; HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* Servo PB1 (TIM3_CH4 AF_PP) */
    GPIO_InitStruct.Pin = GPIO_PIN_1; GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH; HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* Keypad col PA8 */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
    GPIO_InitStruct.Pin = GPIO_PIN_8; GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW; HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* Keypad cols PB13-15 */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_SET);
    GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* Door sensor PA2 (EXTI2, pull-up) */
    GPIO_InitStruct.Pin = DOOR_SW_Pin; GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLUP; HAL_GPIO_Init(DOOR_SW_GPIO_Port, &GPIO_InitStruct);
    HAL_NVIC_SetPriority(EXTI2_IRQn, 5, 0); HAL_NVIC_EnableIRQ(EXTI2_IRQn);

    /* Keypad rows PA9-12 */
    GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT; GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* AS608 TX PB10 (AF_PP) */
    GPIO_InitStruct.Pin = GPIO_PIN_10; GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH; HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* AS608 RX PB11 (input floating) */
    GPIO_InitStruct.Pin = GPIO_PIN_11; GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL; HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|
                                  RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) Error_Handler();
}

void Error_Handler(void) { __disable_irq(); while (1) {} }

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) { (void)file; (void)line; }
#endif
