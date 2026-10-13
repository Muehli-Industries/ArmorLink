"""Exercise the real BLE stream framing without building a full descriptor string."""
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
header = (root / "ArmorLinkBLE.h").read_text()
start = header.index("  template <typename TWriter>\n  bool notifyConfigJsonStreamChunked")
end = header.index("\n  ArmorLinkBleCommandHandler* commandHandler()", start)
method = header[start:end]
source = r'''
#include <ArduinoJson.h>
#include <cassert>
#include <cstring>
#include <string>
#include <vector>
using String = std::string;
struct Print {
 virtual ~Print() = default;
 virtual size_t write(uint8_t) = 0;
 virtual size_t write(const uint8_t* p, size_t n) {
  size_t i = 0; for (; i < n && write(p[i]); ++i) {} return i;
 }
};
struct SerialStub {
 template<class... T> void printf(const char*, T...) {}
 void println(const char*) {}
} Serial;
void delay(unsigned) {}
void yield() {}
struct ArmorLinkBleRuntime {
 bool _clientConnected = true;
 std::vector<String> frames;
 void notifyEventJson(const String& s) { frames.push_back(s); }
''' + method + r'''
};
int main() {
 for (size_t n : {size_t(0), size_t(79), size_t(80), size_t(81), size_t(20000)}) {
  ArmorLinkBleRuntime r;
  const std::string pattern = "abc\"\\\nxyz";
  auto byteAt = [&](size_t i) { return uint8_t(pattern[i % pattern.size()]); };
  assert(r.notifyConfigJsonStreamChunked("Helmet", "", 7, n,
    [&](Print& out) { for (size_t i = 0; i < n; ++i) if (!out.write(byteAt(i))) return false; return true; }, false));
  JsonDocument doc;
  assert(!deserializeJson(doc, r.frames.front()));
  assert(doc["type"] == "config_meta");
  assert(doc["totalLength"].as<size_t>() == n);
  const size_t chunks = n ? (n + 79) / 80 : 1;
  assert(doc["chunkCount"].as<size_t>() == chunks);
  assert(r.frames.size() == chunks + 2);
  std::string reconstructed;
  for (size_t i = 0; i < chunks; ++i) {
   assert(!deserializeJson(doc, r.frames[i + 1]));
   assert(doc["type"] == "config_chunk");
   assert(doc["chunkIndex"].as<size_t>() == i);
   assert(doc["requestId"] == 7);
   reconstructed += doc["data"].as<std::string>();
  }
  assert(reconstructed.size() == n);
  for (size_t i = 0; i < n; ++i) assert(uint8_t(reconstructed[i]) == byteAt(i));
  assert(!deserializeJson(doc, r.frames.back()));
  assert(doc["type"] == "config_end");
 }
 ArmorLinkBleRuntime r;
 assert(!r.notifyConfigJsonStreamChunked("Helmet", "", 8, 100,
   [](Print& out) { out.write(uint8_t('x')); return false; }, false));
 JsonDocument doc;
 assert(!deserializeJson(doc, r.frames.back()));
 assert(doc["type"] != "config_end"); // Failed stream must not report completion.
}
'''
with tempfile.TemporaryDirectory() as directory:
    cpp = Path(directory) / "test.cpp"
    binary = Path(directory) / "test"
    cpp.write_text(source)
    subprocess.run(["c++", "-std=c++17", "-I", str(root.parent / "ArduinoJson" / "src"), str(cpp), "-o", str(binary)], check=True)
    subprocess.run([str(binary)], check=True)
print("BLE config stream: boundaries, escaping, 20 KB stream and writer failure passed")
