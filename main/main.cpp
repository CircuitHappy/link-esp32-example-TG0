#include "driver/gpio.h"
#include "esp_event.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
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

static intr_handle_t s_link_timer_handle;

void high_priority_output_task(void* arg)
{
  auto pLink = static_cast<ableton::Link*>(arg);

  //Reset irq and set for next time
  TIMERG0.int_clr_timers.t0 = 1;
  TIMERG0.hw_timer[0].config.alarm_en = 1;

  //Do the Link task
  bool clockTriggered = false;
  const uint8_t ppqn = 4;
  const double quantum = 4.0;

  auto state = pLink->captureAudioSessionState();
  const auto now = pLink->clock().micros();
  const auto phase = state.phaseAtTime(now, quantum);
  const double clockPhase = (phase - (int)phase) * ppqn;

  if ((clockPhase - (int)clockPhase) < 0.1) {
    clockTriggered = true;
  };

  gpio_set_level(LED, clockTriggered);

  //changing the state
  state.setTempo(138.0, now);

  //causes a crash here
  pLink->commitAudioSessionState(state);


}

void timer_tg0_initialise (int timer_period_us, void *data)
{
    timer_config_t config = {
            .alarm_en = true,				//Alarm Enable
            .counter_en = false,			//If the counter is enabled it will start incrementing / decrementing immediately after calling timer_init()
            .intr_type = TIMER_INTR_LEVEL,	//Is interrupt is triggered on timer’s alarm (timer_intr_mode_t)
            .counter_dir = TIMER_COUNT_UP,	//Does counter increment or decrement (timer_count_dir_t)
            .auto_reload = true,			//If counter should auto_reload a specific initial value on the timer’s alarm, or continue incrementing or decrementing.
            .divider = 80   				//Divisor of the incoming 80 MHz (12.5nS) APB_CLK clock. E.g. 80 = 1uS per timer tick
    };

    timer_init(TIMER_GROUP_0, TIMER_0, &config);
    timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0);
    timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, timer_period_us);
    timer_enable_intr(TIMER_GROUP_0, TIMER_0);
    timer_isr_register(TIMER_GROUP_0, TIMER_0, &high_priority_output_task, data, 0, &s_link_timer_handle);

    timer_start(TIMER_GROUP_0, TIMER_0);
}

void link_task(void* user_param) {
  auto pLink = static_cast<ableton::Link*>(user_param);

  // START HIGH-PRIORITY INTERRUPT
  timer_tg0_initialise(100, user_param);

  while (true) {
    // do low priority things like update UI
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

extern "C" void app_main()
{
  ESP_ERROR_CHECK(nvs_flash_init());
  tcpip_adapter_init();
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  ESP_ERROR_CHECK(example_connect());

  gpio_set_direction(LED, GPIO_MODE_OUTPUT);

  ableton::Link link(120.0f);
  link.enable(true);

  xTaskCreatePinnedToCore(
    link_task, "link_task", 8192, static_cast<void*>(&link), 10, nullptr, 1);

  while (true)
  {
    ableton::link::platform::IoContext::poll();
  }
}
