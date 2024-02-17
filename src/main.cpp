#include <Arduino.h>
#include <ETH.h>

#include <string>

#include "sml2"

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

struct have_fine_time{};
struct pps_pulse{};

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
  Serial.begin(9600);
  Serial2.begin(9600, SERIAL_8N1, IO14, IO15);

  WiFi.onEvent([](WiFiEvent_t ev) { connection.process_event(network_event{ev}); });
  ETH.begin();

  // pinMode(interruptPin, INPUT_PULLUP);
  //  attachInterrupt(digitalPinToInterrupt(interruptPin), button, FALLING);
}

void loop() {   
  delay(100);
  while(Serial2.available()) {
    Serial.print(Serial2.readStringUntil('\n') + '\n');
  }
}