/*
 * 主控MCU - CAN总线控制器
 * 功能：通过CAN总线接收命令，控制从MCU的启动/复位
 * 
 * 硬件平台：ESP32-WROOM-32UE-N16 (U10)
 * 
 * 硬件连接（Pin号 → GPIO号）：
 * - CAN_TX:    Pin31 → GPIO19
 * - CAN_RX:    Pin28 → GPIO17
 * - BOOT_MAIN: Pin25 → GPIO0 (系统使能按键)
 * 
 * 从MCU控制引脚：
 * - BOOT_AUTO:  控制从MCU的GPIO0（HIGH=启动蓝牙）
 * - RESET_AUTO: 控制从MCU的EN引脚（LOW=复位）
 * 
 * 系统使能逻辑：
 *   上电后系统处于"未使能"状态，不响应从MCU控制命令。
 *   通过按下BOOT_MAIN按键或发送CAN使能命令(0x05)激活系统。
 *   激活后按键不可再关闭（单次激活），仅CAN关闭命令(0x06)可关闭。
 *   关闭时自动复位所有从MCU。
 * 
 * 注意：MCU1和MCU2的控制引脚分配在GPIO34/35/36/39（仅输入引脚），
 *       硬件上无法输出控制信号，本固件仅控制MCU3~MCU8。
 */

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/twai.h"
#include "esp_log.h"
#include "nvs_flash.h"

#define TAG "MAIN_CTRL"

// ========== TWAI (CAN) 引脚定义 ==========
#define CAN_TX_PIN   GPIO_NUM_19   // Pin31 → IO19
#define CAN_RX_PIN   GPIO_NUM_17   // Pin28 → IO17

// ========== 系统使能按键 ==========
#define BOOT_MAIN    GPIO_NUM_0    // Pin25 → IO0

// 按键消抖参数
#define DEBOUNCE_MS  50

// ========== 从MCU控制引脚定义（Pin号 → GPIO号）==========
// MCU1: BOOT=GPIO36(仅输入), RESET=GPIO39(仅输入) → 不可控
// MCU2: BOOT=GPIO34(仅输入), RESET=GPIO35(仅输入) → 不可控

#define BOOT3_AUTO   GPIO_NUM_32   // Pin8  → IO32
#define BOOT4_AUTO   GPIO_NUM_25   // Pin10 → IO25
#define BOOT5_AUTO   GPIO_NUM_27   // Pin12 → IO27
#define BOOT6_AUTO   GPIO_NUM_23   // Pin37 → IO23
#define BOOT7_AUTO   GPIO_NUM_21   // Pin33 → IO21
#define BOOT8_AUTO   GPIO_NUM_16   // Pin27 → IO16

#define RESET3_AUTO  GPIO_NUM_33   // Pin9  → IO33
#define RESET4_AUTO  GPIO_NUM_26   // Pin11 → IO26
#define RESET5_AUTO  GPIO_NUM_14   // Pin13 → IO14
#define RESET6_AUTO  GPIO_NUM_22   // Pin36 → IO22
#define RESET7_AUTO  GPIO_NUM_18   // Pin30 → IO18
#define RESET8_AUTO  GPIO_NUM_4    // Pin26 → IO4

// ========== CAN协议定义 ==========
#define CAN_ID_COMMAND       0x100  // 接收命令
#define CAN_ID_STATUS        0x101  // 发送状态反馈

// 命令类型
#define CMD_BOOT_OFF         0x00   // 关闭MCU蓝牙
#define CMD_BOOT_ON          0x01   // 启动MCU蓝牙
#define CMD_RESET            0x02   // 复位MCU
#define CMD_RESET_BOOT       0x03   // 复位后启动
#define CMD_QUERY_STATUS     0x04   // 查询状态
#define CMD_SYSTEM_ENABLE    0x05   // 系统使能
#define CMD_SYSTEM_DISABLE   0x06   // 系统关闭（自动复位所有从MCU）

// ========== MCU数量与可控范围 ==========
#define MCU_TOTAL             8
#define MCU_FIRST_CTRL        2      // 从索引2开始可控（即MCU3）
#define MCU_CONTROLLABLE_MASK 0xFC   // bit2~bit7有效（MCU3~MCU8）

// ========== 全局变量 ==========
static const gpio_num_t boot_pins[MCU_TOTAL] = {
    GPIO_NUM_NC, GPIO_NUM_NC,   // MCU1, MCU2: 不可控
    BOOT3_AUTO,  BOOT4_AUTO,
    BOOT5_AUTO,  BOOT6_AUTO,
    BOOT7_AUTO,  BOOT8_AUTO
};

static const gpio_num_t reset_pins[MCU_TOTAL] = {
    GPIO_NUM_NC, GPIO_NUM_NC,   // MCU1, MCU2: 不可控
    RESET3_AUTO, RESET4_AUTO,
    RESET5_AUTO, RESET6_AUTO,
    RESET7_AUTO, RESET8_AUTO
};

// MCU状态位掩码（bit0=MCU1, ..., bit7=MCU8）
static uint8_t mcu_status = 0x00;

// 系统使能状态
static volatile bool system_enabled = false;

/* ===================== 系统使能控制 ===================== */

/**
 * @brief 激活系统，开始接受从MCU控制命令
 */
static void system_enable(void)
{
    if (system_enabled)
    {
        ESP_LOGW(TAG, "系统已处于使能状态");
        return;
    }
    
    system_enabled = true;
    ESP_LOGI(TAG, "========== 系统已使能 ==========");
    ESP_LOGI(TAG, "开始接受从MCU控制命令");
}

/**
 * @brief 关闭系统，自动复位所有从MCU
 */
static void system_disable(void)
{
    if (!system_enabled)
    {
        ESP_LOGW(TAG, "系统已处于关闭状态");
        return;
    }
    
    ESP_LOGI(TAG, "========== 系统关闭中 ==========");
    
    // 关闭所有BOOT信号
    for (int i = MCU_FIRST_CTRL; i < MCU_TOTAL; i++)
    {
        gpio_set_level(boot_pins[i], 0);
    }
    
    // 复位所有从MCU
    for (int i = MCU_FIRST_CTRL; i < MCU_TOTAL; i++)
    {
        gpio_set_level(reset_pins[i], 0);
    }
    vTaskDelay(pdMS_TO_TICKS(100));
    for (int i = MCU_FIRST_CTRL; i < MCU_TOTAL; i++)
    {
        gpio_set_level(reset_pins[i], 1);
    }
    
    mcu_status = 0x00;
    system_enabled = false;
    
    ESP_LOGI(TAG, "所有从MCU已复位，系统已关闭");
}

/* ===================== 按键处理 ===================== */

/**
 * @brief 初始化BOOT_MAIN按键
 *        GPIO0上电时被用于boot模式选择，之后可作为普通输入
 */
static void button_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << BOOT_MAIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    
    ESP_LOGI(TAG, "BOOT_MAIN按键初始化完成 (GPIO%d)", BOOT_MAIN);
}

/**
 * @brief 按键轮询任务（带消抖）
 *        按下一次激活系统，之后按键不再响应
 */
static void button_task(void *arg)
{
    bool last_level = true;   // 上拉，未按下=HIGH
    
    while (1)
    {
        // 已使能后不再需要检测按键
        if (system_enabled)
        {
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }
        
        bool current_level = gpio_get_level(BOOT_MAIN);
        
        // 检测下降沿（按下，GPIO0按键按下为LOW）
        if (last_level && !current_level)
        {
            // 消抖
            vTaskDelay(pdMS_TO_TICKS(DEBOUNCE_MS));
            current_level = gpio_get_level(BOOT_MAIN);
            
            if (!current_level)
            {
                system_enable();
                
                // 等待按键释放
                while (!gpio_get_level(BOOT_MAIN))
                {
                    vTaskDelay(pdMS_TO_TICKS(10));
                }
            }
        }
        
        last_level = current_level;
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

/* ===================== MCU控制函数 ===================== */

/**
 * @brief 初始化所有可控的控制引脚
 */
static void mcu_control_init(void)
{
    for (int i = MCU_FIRST_CTRL; i < MCU_TOTAL; i++)
    {
        // 先写默认电平，再切换为输出，避免方向切换瞬间产生毛刺
        gpio_reset_pin(boot_pins[i]);
        gpio_set_level(boot_pins[i], 0);
        gpio_set_pull_mode(boot_pins[i], GPIO_PULLDOWN_ONLY);
        gpio_set_direction(boot_pins[i], GPIO_MODE_OUTPUT);

        // RESET为低有效：上电阶段优先保持高电平，避免从MCU被误复位
        gpio_reset_pin(reset_pins[i]);
        gpio_set_level(reset_pins[i], 1);
        gpio_set_pull_mode(reset_pins[i], GPIO_PULLUP_ONLY);
        gpio_set_direction(reset_pins[i], GPIO_MODE_OUTPUT);
    }

    ESP_LOGI(TAG, "MCU控制引脚初始化完成（MCU3~MCU8）");
    ESP_LOGW(TAG, "MCU1, MCU2 不可控（引脚为input-only）");
}

/**
 * @brief 过滤掩码，只保留可控的MCU位
 */
static uint8_t filter_mask(uint8_t mcu_mask)
{
    uint8_t filtered = mcu_mask & MCU_CONTROLLABLE_MASK;
    if (filtered != mcu_mask)
    {
        ESP_LOGW(TAG, "掩码0x%02X包含不可控MCU，已过滤为0x%02X", mcu_mask, filtered);
    }
    return filtered;
}

static void mcu_boot_on(uint8_t mcu_mask)
{
    mcu_mask = filter_mask(mcu_mask);
    
    for (int i = MCU_FIRST_CTRL; i < MCU_TOTAL; i++)
    {
        if (mcu_mask & (1 << i))
        {
            gpio_set_level(boot_pins[i], 1);
            mcu_status |= (1 << i);
            ESP_LOGI(TAG, "MCU%d 蓝牙启动", i + 1);
        }
    }
}

static void mcu_boot_off(uint8_t mcu_mask)
{
    mcu_mask = filter_mask(mcu_mask);
    
    for (int i = MCU_FIRST_CTRL; i < MCU_TOTAL; i++)
    {
        if (mcu_mask & (1 << i))
        {
            gpio_set_level(boot_pins[i], 0);
            mcu_status &= ~(1 << i);
            ESP_LOGI(TAG, "MCU%d 蓝牙关闭", i + 1);
        }
    }
}

static void mcu_reset(uint8_t mcu_mask)
{
    mcu_mask = filter_mask(mcu_mask);
    
    ESP_LOGI(TAG, "复位MCU (mask=0x%02X)", mcu_mask);
    
    for (int i = MCU_FIRST_CTRL; i < MCU_TOTAL; i++)
    {
        if (mcu_mask & (1 << i))
        {
            gpio_set_level(reset_pins[i], 0);
        }
    }
    
    vTaskDelay(pdMS_TO_TICKS(100));
    
    for (int i = MCU_FIRST_CTRL; i < MCU_TOTAL; i++)
    {
        if (mcu_mask & (1 << i))
        {
            gpio_set_level(reset_pins[i], 1);
            ESP_LOGI(TAG, "MCU%d 复位完成", i + 1);
        }
    }
}

static void mcu_reset_and_boot(uint8_t mcu_mask)
{
    mcu_mask = filter_mask(mcu_mask);
    
    ESP_LOGI(TAG, "复位并启动MCU (mask=0x%02X)", mcu_mask);
    
    mcu_boot_on(mcu_mask);
    vTaskDelay(pdMS_TO_TICKS(10));
    mcu_reset(mcu_mask);
    
    ESP_LOGI(TAG, "MCU已复位，将自动启动蓝牙");
}

/* ===================== CAN总线通信 ===================== */

static esp_err_t can_init(void)
{
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
        CAN_TX_PIN, 
        CAN_RX_PIN, 
        TWAI_MODE_NORMAL
    );
    g_config.rx_queue_len = 10;

    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    esp_err_t ret = twai_driver_install(&g_config, &t_config, &f_config);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "TWAI驱动安装失败: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = twai_start();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "TWAI启动失败: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "CAN总线初始化完成 (500Kbps)");
    return ESP_OK;
}

/**
 * @brief 发送CAN状态消息
 *        data[0] = mcu_status (从MCU蓝牙状态)
 *        data[1] = system_enabled (系统使能状态)
 */
static esp_err_t can_send_status(void)
{
    static uint32_t tx_fail_count = 0;

    twai_status_info_t can_status;
    if (twai_get_status_info(&can_status) == ESP_OK && can_status.state == TWAI_STATE_BUS_OFF)
    {
        ESP_LOGE(TAG, "CAN总线BUS-OFF，无法发送状态，尝试恢复");
        twai_initiate_recovery();
        return ESP_ERR_INVALID_STATE;
    }

    twai_message_t tx_msg = {
        .identifier = CAN_ID_STATUS,
        .data_length_code = 2,
        .data = {mcu_status, system_enabled ? 0x01 : 0x00}
    };

    esp_err_t ret = twai_transmit(&tx_msg, pdMS_TO_TICKS(100));
    if (ret == ESP_OK)
    {
        tx_fail_count = 0;
        ESP_LOGD(TAG, "状态发送: mcu=0x%02X, sys=%s", 
                 mcu_status, system_enabled ? "ON" : "OFF");
    }
    else
    {
        tx_fail_count++;
        if (tx_fail_count == 1 || (tx_fail_count % 6) == 0)
        {
            ESP_LOGW(TAG, "状态发送失败(%lu): %s", (unsigned long)tx_fail_count, esp_err_to_name(ret));
        }
    }

    return ret;
}

static void can_process_command(const twai_message_t *msg)
{
    if (msg->data_length_code < 1)
    {
        ESP_LOGW(TAG, "CAN消息长度不足");
        return;
    }

    uint8_t cmd = msg->data[0];

    // 系统使能/关闭和状态查询：任何时候都响应
    switch (cmd)
    {
    case CMD_SYSTEM_ENABLE:
        system_enable();
        can_send_status();
        return;

    case CMD_SYSTEM_DISABLE:
        system_disable();
        can_send_status();
        return;

    case CMD_QUERY_STATUS:
        can_send_status();
        return;

    default:
        break;
    }

    // 以下命令需要系统使能后才处理
    if (!system_enabled)
    {
        ESP_LOGW(TAG, "系统未使能，忽略命令0x%02X（先发送0x05或按BOOT_MAIN）", cmd);
        return;
    }

    if (msg->data_length_code < 2)
    {
        ESP_LOGW(TAG, "CAN消息长度不足（需要2字节）");
        return;
    }

    uint8_t mcu_mask = msg->data[1];
    ESP_LOGI(TAG, "CAN命令: cmd=0x%02X, mask=0x%02X", cmd, mcu_mask);

    switch (cmd)
    {
    case CMD_BOOT_ON:
        mcu_boot_on(mcu_mask);
        break;

    case CMD_BOOT_OFF:
        mcu_boot_off(mcu_mask);
        break;

    case CMD_RESET:
        mcu_reset(mcu_mask);
        break;

    case CMD_RESET_BOOT:
        mcu_reset_and_boot(mcu_mask);
        break;

    default:
        ESP_LOGW(TAG, "未知命令: 0x%02X", cmd);
        break;
    }
}

static void can_rx_task(void *arg)
{
    twai_message_t rx_msg;

    while (1)
    {
        esp_err_t ret = twai_receive(&rx_msg, pdMS_TO_TICKS(1000));
        
        if (ret == ESP_OK)
        {
            if (rx_msg.identifier == CAN_ID_COMMAND)
            {
                can_process_command(&rx_msg);
            }
        }
        else if (ret != ESP_ERR_TIMEOUT)
        {
            ESP_LOGW(TAG, "CAN接收错误: %s", esp_err_to_name(ret));
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}

static void status_monitor_task(void *arg)
{
    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(5000));

        if (system_enabled)
        {
            can_send_status();
        }

        ESP_LOGI(TAG, "状态: sys=%s, mcu=0x%02X", 
                 system_enabled ? "ON" : "OFF", mcu_status);
    }
}

/* ===================== 主函数 ===================== */

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "  主控MCU - CAN总线控制器");
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "  可控: MCU3~MCU8 (6路)");
    ESP_LOGI(TAG, "  不可控: MCU1, MCU2 (input-only引脚)");
    ESP_LOGI(TAG, "  CAN波特率: 500Kbps");
    ESP_LOGI(TAG, "  系统状态: 未使能（等待激活）");
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "");

    // 初始化按键
    button_init();

    // 启动后打印BOOT_MAIN电平，辅助判断GPIO0是否被外部电路持续拉低
    int boot_level = gpio_get_level(BOOT_MAIN);
    ESP_LOGI(TAG, "BOOT_MAIN当前电平: %d (0=按下/被拉低, 1=释放)", boot_level);
    if (boot_level == 0)
    {
        ESP_LOGW(TAG, "检测到GPIO0为低电平：可能影响正常启动，请检查外部上拉与按键电路");
    }

    // 初始化MCU控制引脚
    mcu_control_init();

    // 初始化CAN总线
    ret = can_init();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "CAN初始化失败，系统停止");
        while (1)
        {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    // 创建任务
    xTaskCreate(can_rx_task, "can_rx", 4096, NULL, 5, NULL);
    xTaskCreate(button_task, "button", 2048, NULL, 4, NULL);
    xTaskCreate(status_monitor_task, "status", 2048, NULL, 3, NULL);

    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "系统就绪 - 等待激活");
    ESP_LOGI(TAG, "  按下 BOOT_MAIN 按键 或 发送CAN命令激活");
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "CAN命令格式 (ID=0x100):");
    ESP_LOGI(TAG, "  系统使能:       [0x05]");
    ESP_LOGI(TAG, "  系统关闭:       [0x06]");
    ESP_LOGI(TAG, "  启动MCU3~8:     [0x01, 0xFC]");
    ESP_LOGI(TAG, "  关闭MCU3~8:     [0x00, 0xFC]");
    ESP_LOGI(TAG, "  复位MCU3:       [0x02, 0x04]");
    ESP_LOGI(TAG, "  复位后启动MCU4: [0x03, 0x08]");
    ESP_LOGI(TAG, "  查询状态:       [0x04, 0xFF]");
    ESP_LOGI(TAG, "");
}
