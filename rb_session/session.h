#pragma once

#include <flatbuffers/flatbuffers.h>
#include <memory>
#include <string>
#include <string_view>
#include <iostream>
#include "fbs_helpers.h"
#include "session_backend.h"
#include "session_options.h"

namespace rb::io {

class Session {
 public:
  static Session Open(SessionOptions options);

  static constexpr std::size_t kBufferLength = 1024;

  std::string Resource(std::string_view path) const;

  template <typename FbsRootT, typename BuildFn>
  void Publish(std::string_view path, BuildFn build) {
    flatbuffers::FlatBufferBuilder fbb(kBufferLength);
    auto offset = build(fbb);
    static_assert(std::is_same_v<decltype(offset), flatbuffers::Offset<FbsRootT>>,
                  "build(fbb) must return flatbuffers::Offset<FbsRootT>");
    fbb.Finish(offset);
    backend_->Publish(Resource(path), {fbb.GetBufferPointer(), fbb.GetSize()});
  }

  template <typename FbsRootT>
  void Publish(std::string_view path, const fb::NativeT<FbsRootT>& obj) {
    static_assert(fb::has_native_table_type<FbsRootT>::value, "Enable flatc --gen-object-api for FbsRootT");
    flatbuffers::FlatBufferBuilder fbb(kBufferLength);
    auto off = FbsRootT::Pack(fbb, &obj);
    fbb.Finish(off);
    backend_->Publish(Resource(path), {fbb.GetBufferPointer(), fbb.GetSize()});
  }

  template <typename FbsRootT>
  void Publish(std::string_view path, fb::NativeT<FbsRootT>&& obj) {
    Publish<FbsRootT>(path, obj);
  }

  template <typename FbsRootT>
  void Subscribe(std::string_view path, std::function<void(const FbsRootT*)> cb) {
    backend_->Subscribe(Resource(path), [cb = std::move(cb)](std::span<const uint8_t> raw) {
      if (!rb::fb::Verify<FbsRootT>(raw)) {
        return;
      }
      cb(rb::fb::Get<FbsRootT>(raw));
    });
  }

  template <typename FbsRootT>
  void Subscribe(std::string_view path, std::function<void(fb::NativeT<FbsRootT>)> cb) {
    static_assert(fb::has_native_table_type<FbsRootT>::value, "Enable flatc --gen-object-api for FbsRootT");
    backend_->Subscribe(Resource(path), [cb = std::move(cb)](std::span<const uint8_t> raw) {
      if (!rb::fb::Verify<FbsRootT>(raw))
        return;
      const FbsRootT* root = flatbuffers::GetRoot<FbsRootT>(raw.data());
      fb::NativeT<FbsRootT> obj;
      root->UnPackTo(&obj);
      cb(std::move(obj));
    });
  }

  // 근준 오리지날
  // template <typename ReqRootT, typename RepRootT, typename BuildReqFn, typename UseFn>
  // bool CallWith(std::string_view path, BuildReqFn build_req, UseFn&& cb, int timeout_ms = 100) {
  //   flatbuffers::FlatBufferBuilder fbb(kBufferLength);
  //   auto offset = build_req(fbb);
  //   static_assert(std::is_same_v<decltype(offset), flatbuffers::Offset<ReqRootT>>,
  //                 "build_req(fbb) must return flatbuffers::Offset<ReqRootT>");
  //   fbb.Finish(offset);
  //   auto rep = backend_->Query(Resource(path), {fbb.GetBufferPointer(), fbb.GetSize()}, timeout_ms);
  //   if (rep.empty() || !rb::fb::Verify<RepRootT>(rep)) {
  //     return false;
  //   }
  //   const auto* res = rb::fb::Get<RepRootT>(rep);
  //   if (!res) {
  //     return false;
  //   }
  //   std::invoke(std::forward<UseFn>(cb), res);
  //   return true;
  // }
  template <typename ReqRootT, typename RepRootT, typename BuildReqFn, typename UseFn>
  int CallWith(std::string_view path, BuildReqFn build_req, UseFn&& cb, int timeout_ms = 100) {
      flatbuffers::FlatBufferBuilder fbb(kBufferLength);
      auto offset = build_req(fbb);
      static_assert(std::is_same_v<decltype(offset), flatbuffers::Offset<ReqRootT>>,
                    "build_req(fbb) must return flatbuffers::Offset<ReqRootT>");
      fbb.Finish(offset);

      std::vector<uint8_t> rep;

      try {
          // Zenoh 쿼리 호출
          rep = backend_->Query(Resource(path), {fbb.GetBufferPointer(), fbb.GetSize()}, timeout_ms);
      } catch (const std::runtime_error& e) {
          // 예외 잡아서 false 반환
          std::cerr << "[Session::CallWith] Zenoh query failed: " << e.what() << std::endl;
          return 1;
      }

      // 응답이 비었거나 FlatBuffer 검증 실패
      if (rep.empty() || !rb::fb::Verify<RepRootT>(rep)) {
          return 2;
      }

      const auto* res = rb::fb::Get<RepRootT>(rep);
      if (!res) {
          return 3;
      }

      std::invoke(std::forward<UseFn>(cb), res);
      return 0;
  }

  template <typename ReqRootT, typename RepRootT, typename BuildRepFn>
  void Serve(std::string_view path, BuildRepFn build_rep) {
    backend_->DeclareQueryable(Resource(path), [build_rep](std::span<const uint8_t> req_bytes) {
      if (!rb::fb::Verify<ReqRootT>(req_bytes)) {
        return std::vector<uint8_t>{};
      }
      auto req = rb::fb::Get<ReqRootT>(req_bytes);
      flatbuffers::FlatBufferBuilder fbb(kBufferLength);
      auto rep_offset = build_rep(fbb, req);
      static_assert(std::is_same_v<decltype(rep_offset), flatbuffers::Offset<RepRootT>>,
                    "build_rep(fbb, req) must return flatbuffers::Offset<RepRootT>");
      fbb.Finish(rep_offset);
      auto p = fbb.Release();
      return std::vector<uint8_t>(p.data(), p.data() + p.size());
    });
  }

 private:
  Session(std::unique_ptr<SessionBackend> backend, std::string ns) : backend_(std::move(backend)), ns_(std::move(ns)) {}

  std::unique_ptr<SessionBackend> backend_;
  std::string ns_;
};

}  // namespace rb::io
