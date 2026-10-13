"""Exercise the actual section collection paths with duplicate system actions."""
from pathlib import Path
import subprocess
import tempfile
header = (Path(__file__).resolve().parents[1] / "ArmorLinkDescriptor.h").read_text()
functions = []
for name, end in [("writeSectionsStreaming", "    bool firstSection"), ("appendSections", "    for (const auto& sectionName")]:
    start = header.index("  static void " + name)
    body_start = header.index("    const bool isWebSerial", start)
    body_end = header.index(end, body_start)
    functions.append("std::vector<String> " + name + "(const Module& module, Profile profile) {\n" + header[body_start:body_end] + "return orderedSections; }\n")
source = r'''#include <cassert>
#include <string>
#include <vector>
#include <algorithm>
#include <cctype>
struct String : std::string {
 using std::string::string;
 String(const std::string& s): std::string(s) {}
 bool isEmpty() const { return empty(); }
 bool equalsIgnoreCase(const char* s) const {
  auto a = std::string(*this), b = std::string(s);
  for (auto& c:a) c=std::tolower(c);
  for (auto& c:b) c=std::tolower(c);
  return a==b;
 }
};
String normalizeId(String s) { for(auto& c:s) c=std::tolower(c); return s; }
struct Item { String section; };
struct Registry {
 std::vector<Item> data;
 const auto& items() const { return data; }
 const auto& customActions() const { return data; }
};
struct Module {
 Registry fields, actions_, custom;
 const auto& config() const { return fields; }
 const auto& actions() const { return actions_; }
 const auto& actionBuilder() const { return custom; }
};
enum class Profile { WebSerial, Ble };
''' + "\n".join(functions) + r'''
int main() {
 for (auto collect : {writeSectionsStreaming, appendSections}) {
  Module m; m.fields.data = {{"ArmorLink"}};
  m.actions_.data = {{"General"}, {"ArmorLink"}};
  assert((collect(m, Profile::WebSerial) == std::vector<String>{"ArmorLink", "General"}));
  m.custom.data = {{"ArmorLink"}, {"Custom"}};
  assert((collect(m, Profile::WebSerial) == std::vector<String>{"ArmorLink", "General", "Custom"}));
  m.actions_.data = {{"General"}}; m.custom.data.clear();
  assert((collect(m, Profile::WebSerial) == std::vector<String>{"ArmorLink", "General"}));
  m.fields.data.clear();
  assert((collect(m, Profile::WebSerial) == std::vector<String>{"General"}));
 }
}
'''
with tempfile.TemporaryDirectory() as directory:
    cpp = Path(directory) / "test.cpp"
    binary = Path(directory) / "test"
    cpp.write_text(source)
    subprocess.run(["c++", "-std=c++17", str(cpp), "-o", str(binary)], check=True)
    subprocess.run([str(binary)], check=True)
print("Descriptor sections: streaming and document paths emit the system section exactly once")
