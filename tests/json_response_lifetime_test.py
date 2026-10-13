"""Verify actual ACK/error handlers release JSON storage before BLE notification."""
from pathlib import Path
import re
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
header = (root / "ArmorLink.h").read_text()
start = header.index("  static void sendAck(", header.index("class ArmorLinkAutoBleCommandHandler"))
end = header.index("  static void sendEventJson", start)
methods = header[start:end].replace("message.isEmpty()", "message.empty()").replace("detail.isEmpty()", "detail.empty()")
methods = re.sub(r"StaticJsonDocument<(\d+)> doc;", r"StaticJsonDocument<\1> doc(&allocator);", methods)
source = r'''
#include <ArduinoJson.h>
#include <cassert>
#include <cstdlib>
#include <map>
#include <string>
#include <vector>
using String = std::string;
struct TrackingAllocator : ArduinoJson::Allocator {
 std::map<void*, size_t> sizes;
 size_t used = 0, peak = 0;
 void* allocate(size_t n) override {
  void* p = malloc(n); if (p) { sizes[p] = n; used += n; if (used > peak) peak = used; } return p;
 }
 void deallocate(void* p) override {
  if (!p) return; used -= sizes.at(p); sizes.erase(p); free(p);
 }
 void* reallocate(void* p, size_t n) override {
  if (!p) return allocate(n);
  const size_t old = sizes.at(p);
  void* q = realloc(p, n);
  if (q) { sizes.erase(p); sizes[q] = n; used = used - old + n; if (used > peak) peak = used; }
  return q;
 }
} allocator;
struct SerialStub { template<class... T> void printf(const char*, T...) {} } Serial;
std::vector<String> frames;
void bleNotifyEventJson(const String& json) {
 assert(allocator.peak > 0);
 assert(allocator.used == 0); // JSON pool/string allocations freed before BLE allocates.
 frames.push_back(json);
}
struct Handler {
''' + methods + r'''
};
int main() {
 Handler::sendAck(42, "ok", "Saved \"eyes\"\nnext line");
 Handler::sendError(43, "Bad value", "Details");
 Handler::sendAck(44, "ok", "");
 Handler::sendError(45, "Bad value", "");
 JsonDocument doc;
 assert(!deserializeJson(doc, frames[0]));
 assert(doc["type"] == "ack" && doc["requestId"] == 42 && doc["status"] == "ok");
 assert(doc["message"] == "Saved \"eyes\"\nnext line");
 assert(!deserializeJson(doc, frames[1]));
 assert(doc["type"] == "error" && doc["requestId"] == 43 && doc["message"] == "Bad value");
 assert(doc["detail"] == "Details");
 assert(!deserializeJson(doc, frames[2]));
 assert(doc["message"].isNull());
 assert(!deserializeJson(doc, frames[3]));
 assert(doc["detail"].isNull());
}
'''
with tempfile.TemporaryDirectory() as directory:
    cpp = Path(directory) / "test.cpp"
    binary = Path(directory) / "test"
    cpp.write_text(source)
    subprocess.run(["c++", "-std=c++17", "-Wno-deprecated-declarations", "-I", str(root.parent / "ArduinoJson" / "src"), str(cpp), "-o", str(binary)], check=True)
    subprocess.run([str(binary)], check=True)
print("JSON response lifetime: ACK/error data preserved; JSON allocations released before notification")
