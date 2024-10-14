#include <zephyr/sys/printk.h>
#include <zephyr/kernel.h>
#include <hal/nrf_gpio.h>  // 引入 GPIO 控制的庫
#include <zephyr/device.h>  // 引入設備管理的庫
#include <zephyr/drivers/gpio.h>  // 引入 GPIO 驅動的庫

#define LED_PIN NRF_GPIO_PIN_MAP(0, 13)     // LED 連接的引腳號
#define SENSOR_PIN NRF_GPIO_PIN_MAP(0, 28)  // 傳感器 D0 連接的引腳號

static struct gpio_callback sensor_cb;   // GPIO 中斷回調結構
static struct k_timer sensor_timer;
static bool interrupt_handling = false;  // 防重入標誌位

void sensor_timer_callback(struct k_timer *timer_id) {
    const struct device *gpio_dev = device_get_binding("GPIO_0");
    gpio_pin_interrupt_configure(gpio_dev, SENSOR_PIN, GPIO_INT_EDGE_RISING);
    interrupt_handling = false;  // 定时器到期后，允许中断再次触发
}

void sensor_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
    if (interrupt_handling) {
        return;  // 如果已经在处理中则直接返回，防止重入
    }

    // 禁用传感器中断，防止再次触发
    gpio_pin_interrupt_configure(dev, SENSOR_PIN, GPIO_INT_DISABLE);
    interrupt_handling = true;  // 設置為處理中，防止重入

    k_msleep(500);  // 去抖動處理
    if (nrf_gpio_pin_read(SENSOR_PIN) == 0) {
        // 启动定时器，200ms 后重新启用中断
        k_timer_start(&sensor_timer, K_MSEC(200), K_NO_WAIT);
        return;
    }

    // 切換 LED 狀態
    static bool led_state = false;
    led_state = !led_state;
    if (led_state) {
        nrf_gpio_pin_clear(LED_PIN);
        printk("LED ON.\n");
    } else {
        nrf_gpio_pin_set(LED_PIN);
        printk("LED OFF.\n");
    }

    // 重启定时器以再次启用中断
    k_timer_start(&sensor_timer, K_MSEC(200), K_NO_WAIT);
}

void setup() {
    const struct device *gpio_dev = device_get_binding("GPIO_0");
    if (!gpio_dev) {
        printk("Error: Cannot find GPIO device!\n");
        return;
    }

    nrf_gpio_cfg_output(LED_PIN);  // 設定 LED 引腳為輸出
    nrf_gpio_pin_set(LED_PIN);     // 初始化時將 LED 設置為關閉

    // 配置傳感器引腳為輸入
    nrf_gpio_cfg_input(SENSOR_PIN, NRF_GPIO_PIN_NOPULL);

    // 設置中斷
    gpio_pin_interrupt_configure(gpio_dev, SENSOR_PIN, GPIO_INT_EDGE_RISING);
    gpio_init_callback(&sensor_cb, sensor_callback, BIT(SENSOR_PIN));
    gpio_add_callback(gpio_dev, &sensor_cb);

    // 初始化定时器，用于限流中断
    k_timer_init(&sensor_timer, sensor_timer_callback, NULL);
}

void main() {
    setup();
    printk("System initialized.\n");

    while (1) {
        k_msleep(1000);  // 主循環中可以執行其他任務或進入低功耗模式
        if (nrf_gpio_pin_read(SENSOR_PIN) == 0) {
            printk("P0.28 = 0\n");
        } else {
            printk("P0.28 = 1\n");
        }
    }
}
