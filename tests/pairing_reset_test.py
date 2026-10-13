"""Compile the actual storage reset method against an isolated Preferences fake."""
from pathlib import Path
import subprocess
import tempfile

header = (Path(__file__).resolve().parents[1] / "ArmorLinkStorage.h").read_text()
start = header.index("  bool clearPairingInfo() {")
end = header.index("\n  size_t loadPairedModules", start)
method = header[start:end]
source = r'''#include <cassert>
#include <map>
#include <string>
using String = std::string;
struct Preferences {
  static inline std::map<std::string, std::string> data;
  static inline bool openOk = true;
  static inline std::string failKey;
  bool begin(const char*, bool) { return openOk; }
  bool isKey(const char* key) { return data.count(key); }
  bool remove(const char* key) {
    if (failKey == key) return false;
    return data.erase(key) != 0;
  }
  void end() {}
};
struct Storage {
  struct Namespace : std::string { bool isEmpty() const { return empty(); } };
  Namespace _namespace;
''' + method + r'''
};
int main() {
  Storage s; s._namespace.assign("armorlink");
  const std::map<std::string, std::string> other = {
    {"brightness", "123"}, {"profile_name", "Helmet"},
    {"profile_id", "custom"}, {"actions", "sequence"},
    {"nodeName", "Helmet"}, {"paired_count", "2"},
    {"pm_name_0", "Hand"}, {"custom_unknown", "keep"}
  };
  Preferences::data = other;
  for (auto key : {"pair_paired", "pair_gw_name", "pair_gw_mac", "pair_rpin"})
    Preferences::data[key] = "old";
  assert(s.clearPairingInfo());
  assert(Preferences::data == other); // All unrelated values survive exactly.
  assert(s.clearPairingInfo()); // Already unpaired is a successful no-op.
  assert(Preferences::data == other);
  Preferences::data["pair_paired"] = "true";
  const auto before = Preferences::data;
  Preferences::openOk = false;
  assert(!s.clearPairingInfo());
  assert(Preferences::data == before);
  Preferences::openOk = true;
  Preferences::failKey = "pair_paired";
  assert(!s.clearPairingInfo());
  assert(Preferences::data == before);
  s._namespace.clear();
  assert(!s.clearPairingInfo());
}
'''
with tempfile.TemporaryDirectory() as directory:
    cpp = Path(directory) / "test.cpp"
    binary = Path(directory) / "test"
    cpp.write_text(source)
    subprocess.run(["c++", "-std=c++17", str(cpp), "-o", str(binary)], check=True)
    subprocess.run([str(binary)], check=True)
print("Pairing reset: unrelated storage preserved, idempotence and failures passed")
