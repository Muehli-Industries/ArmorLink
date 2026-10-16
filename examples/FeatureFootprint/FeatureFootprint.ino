#include <ArmorLink.h>
ArmorLinkModule module("FeatureTest", ArmorLinkModuleType::Generic);
int value = 1;
void setup() {
  Serial.begin(115200);
  module.config().addInt("value", &value, 1).label("Value").persist("value");
  module.actions().add("ping").command("test", "ping").onExecute([] { value++; });
  module.serial().addCommand("ping", [](const String&) { value++; });
  ArmorLinkOptions options;
  options.enableBle = true;
  options.enableGateway = true;
  ArmorLink.begin(module, options);
}
void loop() { ArmorLink.loop(); }
