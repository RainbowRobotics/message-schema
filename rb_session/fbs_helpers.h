#ifndef RB_MIDDLEWARE_SESSION_FBS_HELPERS_H_
#define RB_MIDDLEWARE_SESSION_FBS_HELPERS_H_

#include <flatbuffers/flatbuffers.h>
#include <flatbuffers/verifier.h>
#include <span>

namespace rb::fb {

template <typename T, typename = void>
struct has_native_table_type : std::false_type {};

template <typename T>
struct has_native_table_type<T, std::void_t<typename T::NativeTableType>> : std::true_type {};

template <typename FbsRootT>
using NativeT = typename FbsRootT::NativeTableType;

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