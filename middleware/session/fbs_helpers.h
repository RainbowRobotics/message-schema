#ifndef RB_MIDDLEWARE_SESSION_FBS_HELPERS_H_
#define RB_MIDDLEWARE_SESSION_FBS_HELPERS_H_

#include <flatbuffers/flatbuffers.h>
#include <flatbuffers/verifier.h>
#include <span>

namespace rb::fb {

template <typename RootT>
inline bool Verify(std::span<const uint8_t> raw) {
  flatbuffers::Verifier v(raw.data(), raw.size());
  return v.VerifyBuffer<RootT>(nullptr);
}

template <typename RootT>
inline const RootT* Get(std::span<const uint8_t> raw) {
  return flatbuffers::GetRoot<RootT>(raw.data());
}

}  // namespace rb::fb

#endif