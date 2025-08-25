#include <flatbuffers/flatbuffers.h>
#include <array>
#include <fstream>
#include <iostream>
#include "rb/example/state_generated.h"

using namespace rb::example;

int main(int argc, char** argv) {
  const char* path = (argc > 1) ? argv[1] : "/tmp/state";

  flatbuffers::FlatBufferBuilder fbb;

  State state{flatbuffers::span<const float, 7>({1.f, 2.f, 3.f, 4.f, 5.f, 6.f, 7.f}),
              flatbuffers::span<const float, 7>({7.f, 6.f, 5.f, 4.f, 3.f, 2.f, 1.f})};

  RootStateBuilder rs(fbb);
  rs.add_state(&state);
  auto root = rs.Finish();

  fbb.Finish(root);

  std::ofstream ofs(path, std::ios::binary);
  ofs.write(reinterpret_cast<const char*>(fbb.GetBufferPointer()), fbb.GetSize());
  std::cout << "Wrote " << fbb.GetSize() << " bytes to " << path << "\n";

  return 0;
}