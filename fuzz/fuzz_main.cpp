// Corpus replay driver for builds without libFuzzer (GCC, Apple Clang): runs
// every file named on the command line, or found in a named directory,
// through the same LLVMFuzzerTestOneInput as the fuzzer. Exits non-zero if
// any input aborts (the harness aborts on a broken property).
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <vector>

extern "C" int LLVMFuzzerTestOneInput(const std::uint8_t* data, std::size_t size);

int main(int argc, char** argv) {
  std::vector<std::filesystem::path> files;
  for (int i = 1; i < argc; ++i) {
    const std::filesystem::path p(argv[i]);
    if (std::filesystem::is_directory(p)) {
      for (const auto& e : std::filesystem::directory_iterator(p)) {
        if (e.is_regular_file()) files.push_back(e.path());
      }
    } else {
      files.push_back(p);
    }
  }
  if (files.empty()) {
    std::fprintf(stderr, "usage: %s <corpus dir or files>...\n", argv[0]);
    return 2;
  }
  std::size_t bytes = 0;
  for (const auto& f : files) {
    std::ifstream in(f, std::ios::binary);
    const std::vector<char> raw((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    const std::vector<std::uint8_t> buf(raw.begin(), raw.end());
    LLVMFuzzerTestOneInput(buf.data(), buf.size());
    bytes += buf.size();
  }
  std::printf("fuzz corpus replay: %zu inputs, %zu bytes, all properties held\n", files.size(), bytes);
  return 0;
}
