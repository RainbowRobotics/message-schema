#ifndef RB_MIDDLEWARE_SESSION_SESSION_H_
#define RB_MIDDLEWARE_SESSION_SESSION_H_

#include <flatbuffers/flatbuffers.h>
#include <memory>
#include <string>
#include <string_view>
#include "middleware/session/fbs_helpers.h"
#include "middleware/session/session_backend.h"
#include "middleware/session/session_options.h"

namespace rb::io {

class Session {
 public:
  static Session Open(const SessionOptions& options);

  static constexpr std::size_t kBufferLength = 1024;

  std::string Resource(std::string_view path) const;
  void PublishBuf(std::string_view path, SessionBackend::Bytes buf);

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
  void Subscribe(std::string_view path, std::function<void(const FbsRootT*)> cb) {
    backend_->Subscribe(Resource(path), [cb = std::move(cb)](std::span<const uint8_t> raw) {
      if (!rb::fb::Verify(FbsRootT)(raw)) {
        return;
      }
      cb(rb::fb::Get<FbsRootT>(raw));
    });
  }

  template <typename ReqRootT, typename RepRootT, typename BuildReqFn>
  const ReqRoot* Call(std::string_view path, BuildReqFn build_req, int timeout_ms = 100) {
    flatbuffers::FlatBufferBuilder fbb(kBufferLength);
    auto offset = build_req(fbb);
    static_assert(std::is_same_v<decltype(off), flatbuffers::Offset<ReqRootT>>,
                  "build_req(fbb) must return flatbuffers::Offset<ReqRootT>");
    fbb.Finish(offset);
    auto req = backend_->Query(Resource(path), {fbb.GetBufferPointer(), fbb.GetSize()}, timeout_ms);
    if (rep.empty() || !rb::fb::Verify<RepRootT>(rep)) {
      return nullptr;
    }
    return rb::fb::Get<RepRootT>(rep);
  }

  template <typename ReqRootT, typename BuildRepFn>
  void Serve(std::string_view path, BuildRepFn build_rep) {
    backend_->DeclareQueryable(Resource(path), [build_rep](std::span<const uint8_t> req_bytes) {
      if (!rb::fb::Verify<ReqRootT>(req_bytes)) {
        return std::vector<uint8_t>{};
      }
      auto req = rb::fb::Get<ReqRootT>(req_bytes);
      flatbuffers::FlatBufferBuilder fbb(kBufferLength);
      auto rep_offset = build_rep(fbb, req);
      fbb.Finish(rep_offset);
      auto p = fbb.Release();
      return std::vector<uint8_t>(p.data(), p.data() + p.size());
    });
  }

 private:
  Session(std::shared_ptr<SessionBackend> backend, std::string ns)
      : backend_(std::move(backend)), ns_(std::move(ns)) {}

  std::shared_ptr<SessionBackend> backend_;
  std::string ns_;
};

}  // namespace rb::io

#endif