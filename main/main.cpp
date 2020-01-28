#include "driver/gpio.h"
#include "esp_event.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/timer.h"
#include "nvs_flash.h"
#include "protocol_examples_common.h"
#include <ableton/Link.hpp>

#define LED GPIO_NUM_18

unsigned int if_nametoindex(const char* ifname)
{
  return 0;
}

char* if_indextoname(unsigned int ifindex, char* ifname)
{
  return nullptr;
}

void IRAM_ATTR timer_HighPriority_isr(void* user_param)
{
  static BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  TIMERG0.int_clr_timers.t0 = 1;
  TIMERG0.hw_timer[0].config.alarm_en = 1;

  xSemaphoreGiveFromISR(user_param, &xHigherPriorityTaskWoken);
  if (xHigherPriorityTaskWoken)
  {
    portYIELD_FROM_ISR();
  }
}


void timer_group0_init(int timer_period_us, void* user_param)
{
    timer_config_t config = {
            .alarm_en = true,
            .counter_en = false,
            .intr_type = TIMER_INTR_LEVEL,
            .counter_dir = TIMER_COUNT_UP,
            .auto_reload = true,
            .divider = 80
    };

    timer_init(TIMER_GROUP_0, TIMER_0, &config);
    timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0);
    timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, timer_period_us);
    timer_enable_intr(TIMER_GROUP_0, TIMER_0);
    timer_isr_register(TIMER_GROUP_0, TIMER_0, &timer_HighPriority_isr, user_param, 0, nullptr);
    timer_start(TIMER_GROUP_0, TIMER_0);
}

void low_priority_task(void* user_param)
{
  auto link = static_cast<ableton::Link*>(user_param);
  double quantum = 4.0;

  while (true)
  {
    auto state = link->captureAppSessionState();
    std::size_t numPeers = link->numPeers();
    auto now = link->clock().micros();
    auto beats = state.beatAtTime(now, quantum);
    auto phase = state.phaseAtTime(now, quantum);
    auto tempo = state.tempo();

    printf("Peers: %d | tempo: %d | beats: %f | ", numPeers, (int)tempo, beats);
    for (int i = 0; i < ceil(quantum); ++i)
    {
      if (i < phase)
      {
        printf("X");
      }
      else
      {
        printf("O");
      }
    }
    printf("\n");
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

void high_priority_task(void* user_param)
{
  ableton::Link link(120.0f);
  link.enable(true);

  gpio_set_direction(LED, GPIO_MODE_OUTPUT);

  xTaskCreatePinnedToCore(
    low_priority_task, "low-priority", 8192, &link, 1, nullptr, 1);

  while (true)
  {
    if (xSemaphoreTake(user_param, portMAX_DELAY) == pdTRUE)
    {
      const auto state = link.captureAudioSessionState();
      const auto phase = state.phaseAtTime(link.clock().micros(), 1.);
      gpio_set_level(LED, fmodf(phase, 1.) < 0.1);
    }
  }
}

extern "C" void app_main()
{
  ESP_ERROR_CHECK(nvs_flash_init());
  tcpip_adapter_init();
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  ESP_ERROR_CHECK(example_connect());

  SemaphoreHandle_t hi_priority_semphr = xSemaphoreCreateBinary();
  timer_group0_init(100, hi_priority_semphr);

  xTaskCreatePinnedToCore(
    high_priority_task, "hi-priority", 8192, hi_priority_semphr, 1, nullptr, 1);

  while (true)
  {
    ableton::link::platform::IoContext::poll();
    vTaskDelay(1);
  }
}
