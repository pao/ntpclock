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

template <WiFiEvent_t ev>
bool when(const network_event& net) {
  return net.ev == ev;
}

sml::sm sys = [] {
  using namespace sml::dsl;
  return transition_table{
      // clang-format off
      *"eth_uninitialized"_s + event<network_event>[&when<ARDUINO_EVENT_ETH_START>] / start_network = "eth_disconnected"_s,
       "eth_disconnected"_s + event<network_event>[&when<ARDUINO_EVENT_ETH_CONNECTED>] / connect_network = "eth_connected"_s,
       "eth_connected"_s + event<network_event>[&when<ARDUINO_EVENT_ETH_DISCONNECTED>] / disconnect_network = "eth_disconnected"_s,
       "eth_ready"_s + event<network_event>[&when<ARDUINO_EVENT_ETH_DISCONNECTED>] / disconnect_network = "eth_disconnected"_s,
      // clang-format on
  };
};

auto interruptPin = 4;

// void button() { blinker.process_event(button_pressed{}); }

// Ethernet events are handled in the same handler as wifi events
void dispatch_network_event(WiFiEvent_t event) {
  switch (event) {
    // case ARDUINO_EVENT_ETH_GOT_IP:
    //   Serial.print("ETH MAC: ");
    //   Serial.print(ETH.macAddress());
    //   Serial.print(", IPv4: ");
    //   Serial.print(ETH.localIP());
    //   if (ETH.fullDuplex()) {
    //     Serial.print(", FULL_DUPLEX");
    //   }
    //   Serial.print(", ");
    //   Serial.print(ETH.linkSpeed());
    //   Serial.println("Mbps");
    //   eth_connected = true;
    //   break;
  }
}

void setup() {
  Serial.begin(9600);

  WiFi.onEvent([](WiFiEvent_t ev) { sys.process_event(network_event{ev}); });
  ETH.begin();

  pinMode(interruptPin, INPUT_PULLUP);
  // attachInterrupt(digitalPinToInterrupt(interruptPin), button, FALLING);
}

void loop() {
  // Serial.println("Hello world!");
  // sys.process_event(toggle{});
  delay(1000);
}