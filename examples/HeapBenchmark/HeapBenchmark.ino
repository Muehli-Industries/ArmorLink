#include <ArmorLink.h>
#include <esp_heap_caps.h>
#ifndef ARMORLINK_HEAP_VARIANT
#define ARMORLINK_HEAP_VARIANT "default"
#endif
ArmorLinkModule module("HeapBenchmark", ArmorLinkModuleType::Generic);
int value = 1;
void sample(const char* phase) {
  Serial.printf("@ALHEAP:{\"variant\":\"%s\",\"phase\":\"%s\",\"free\":%u,\"largest\":%u,\"minimum\":%u}\n",
    ARMORLINK_HEAP_VARIANT, phase,
    (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
    (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
    (unsigned)heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
}
void settle() {
  for (int i=0; i<300; ++i) { ArmorLink.loop(); delay(10); }
}
class DiscardPrint : public Print {
public:
  size_t write(uint8_t) override { return 1; }
  size_t write(const uint8_t*, size_t n) override { return n; }
};
void setup() {
  Serial.begin(115200);
  delay(2000);
  sample("before_begin");
  module.config().addInt("value", &value, 1).label("Value").persist("value");
  module.actions().add("ping").command("test", "ping").onExecute([] { value++; });
  module.serial().addCommand("ping", [](const String&) { value++; });
  ArmorLinkOptions options;
  options.enableBle = true;
  options.enableGateway = true;
  options.nvsNamespace = "al_heaptest";
  options.requestStateSyncAfterBoot = false;
  ArmorLink.begin(module, options);
  sample("after_begin");
  settle();
  sample("idle");
#if ARMORLINK_HAS_ACTIONBUILDER
  for (int i=0; i<8; ++i) {
    module.actionBuilder().addComponent(String("state") + i, ArmorLinkActionComponentKind::State);
    ArmorLinkActionDefinition definition;
    definition.entity = "heap";
    definition.command = String("sequence") + i;
    definition.label = "Benchmark sequence";
    for (int j=0; j<16; ++j) {
      ArmorLinkActionOp op;
      op.code = ArmorLinkActionOpCode::Wait;
      op.argument = 10;
      definition.ops.push_back(op);
    }
    module.actionBuilder().setDefinition(definition);
  }
#endif
  sample("populated");
#if ARMORLINK_HAS_ACTIONBUILDER
  module.actionBuilder().execute("heap", "sequence0", module.actions());
#endif
  sample("active_sequence");
  settle();
  DiscardPrint sink;
  bool ok = true;
  for (int i=0; i<100; ++i) {
    ok = ArmorLinkDescriptor::writeBle(module, sink) && ok;
    ArmorLink.loop();
  }
  settle();
  sample("after_workload");
  Serial.printf("@ALHEAP_DONE:%s:%s\n", ARMORLINK_HEAP_VARIANT, ok ? "ok" : "failed");
}
void loop() { delay(100); }
