#include <Arduino.h>
#include <ETH.h>

#include <chrono>
#include <string>

#include "TinyGPSPlus.h"
#include "sml2"

template <typename Duration>
auto increment_time(TinyGPSTime gpsTime, Duration increment) {
  using namespace std::chrono;
  return hh_mm_ss{hours{gpsTime.hour()} + minutes{gpsTime.minute()} +
                  seconds{gpsTime.second()} + increment};
}

template <typename Duration>
auto increment_date(TinyGPSDate gpsDate, Duration increment) {
  using namespace std::chrono;
  return year_month_day{sys_days{year{gpsDate.year()} / month{gpsDate.month()} /
                                 day{gpsDate.day()}} +
                        increment};
}

struct network_event {
  WiFiEvent_t ev{};
};

auto start_network = [] {
  Serial.println("ETH Started");
  ETH.setHostname("h2");
};

auto connect_network = [] { Serial.println("ETH Connected"); };
auto disconnect_network = [] { Serial.println("ETH Disconnected"); };

auto got_ip = [] {
  Serial.print("ETH MAC: ");
  Serial.print(ETH.macAddress());
  Serial.print(", IPv4: ");
  Serial.print(ETH.localIP());
  if (ETH.fullDuplex()) {
    Serial.print(", FULL_DUPLEX");
  }
  Serial.print(", ");
  Serial.print(ETH.linkSpeed());
  Serial.println("Mbps");
};

template <WiFiEvent_t ev>
struct when {
  constexpr bool operator()(const network_event& net) const {
    return net.ev == ev;
  }
};

sml::sm connection = [] {
  using namespace sml::dsl;
  return transition_table{
      // clang-format off
    *"uninitialized"_s + event<network_event>[when<ARDUINO_EVENT_ETH_START>{}] / start_network = "disconnected"_s,
     "disconnected"_s + event<network_event>[when<ARDUINO_EVENT_ETH_CONNECTED>{}] / connect_network = "connected"_s,
     "connected"_s + event<network_event>[when<ARDUINO_EVENT_ETH_DISCONNECTED>{}] / disconnect_network = "disconnected"_s,
     "connected"_s + event<network_event>[when<ARDUINO_EVENT_ETH_GOT_IP>{}] / got_ip = "ready"_s,
     "ready"_s + event<network_event>[when<ARDUINO_EVENT_ETH_DISCONNECTED>{}] / disconnect_network = "disconnected"_s,
      // clang-format on
  };
};

volatile SemaphoreHandle_t ppsSemaphore{};
TinyGPSPlus gnss{};
struct have_fine_time {};
struct pps_pulse {};

auto pps = [] { xSemaphoreGiveFromISR(ppsSemaphore, nullptr); };

/*
sml::sm timesync = [] {
  using namespace sml::dsl;
  return transition_table{
    // clang-format off
    *"init"_s + event<
    // clang-format on
  };
};
*/

void setup() {
  // Debug console setup
  Serial.begin(9600);

  // GNSS module setup
  Serial2.begin(9600, SERIAL_8N1, IO14, IO15);
  Serial2.print("$PMTK251,38400*27\r\n");
  Serial2.updateBaudRate(38400);
  delay(100);
  Serial2.print("$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n");
  Serial2.print("$PMTK255,1*2D\r\n");

  // ETH setup
  WiFi.onEvent(
      [](WiFiEvent_t ev) { connection.process_event(network_event{ev}); });
  ETH.begin();

  // PPS setup
  constexpr auto pps_pin = IO36;
  ppsSemaphore = xSemaphoreCreateBinary();
  pinMode(pps_pin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pps_pin), pps, FALLING);
}

void loop() {
  using namespace std::chrono;
  using namespace std::chrono_literals;
  if (xSemaphoreTake(ppsSemaphore, 0) == pdTRUE) {
    if (gnss.sentencesWithFix() > 0 && gnss.time.isValid()) {
      auto display_time = increment_time(gnss.time, 1s);
      auto display_date = increment_date(
          gnss.date, (gnss.time.hour() < display_time.hours().count())
                         ? days{1}
                         : days{0});
      Serial.printf("corrected:   %04d-%02d-%02dT%02lld:%02lld:%02lld\n", int{display_date.year()},
                    unsigned{display_date.month()}, unsigned{display_date.day()},
                    display_time.hours().count(), display_time.minutes().count(),
                    display_time.seconds().count());
      Serial.printf("uncorrected: %04d-%02d-%02dT%02d:%02d:%02d\n", gnss.date.year(),
                    gnss.date.month(), gnss.date.day(), gnss.time.hour(),
                    gnss.time.minute(), gnss.time.second());
    }
    Serial2.flush();
    delay(800);
    while (Serial2.available()) {
      gnss.encode(Serial2.read());
    }
  }
}