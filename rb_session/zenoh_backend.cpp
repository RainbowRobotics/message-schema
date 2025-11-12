#include "zenoh_backend.h"
#include <zenoh.h>
#include <mutex>
#include <iostream>

namespace rb::io {

struct ZenohBackend::Impl {
  ZenohConfig config_;
  z_owned_session_t session_{};

  std::mutex mu_;
  std::vector<z_owned_subscriber_t> subs_;
  std::vector<z_owned_queryable_t> qbls_;

  Impl(ZenohConfig config) : config_(std::move(config)) {
    if (z_open(&session_, z_move(config_.config_), NULL) != 0) {
      throw std::runtime_error("zenoh: z_open failed");
    }
  }

  ~Impl() {
    {
      std::scoped_lock lk(mu_);
      for (auto& q : qbls_) {
        z_drop(z_move(q));
      }
      for (auto& s : subs_) {
        z_drop(z_move(s));
      }
      qbls_.clear();
      subs_.clear();
    }
    z_drop(z_move(session_));
  }

  void Publish(std::string_view resource, Bytes bytes) {
    z_owned_keyexpr_t keyexpr{};
    size_t len = resource.size();
    if (z_keyexpr_from_substr_autocanonize(&keyexpr, resource.data(), &len) < 0) {
      throw std::runtime_error("zenoh: invalid keyexpr");
    }

    z_owned_bytes_t payload{};
    if (z_bytes_copy_from_buf(&payload, bytes.data(), bytes.size()) < 0) {
      z_keyexpr_drop(z_move(keyexpr));
      throw std::runtime_error("zenoh: bytes_copy_from_buf failed");
    }

    z_put_options_t opts{};
    z_put_options_default(&opts);
    if (z_put(z_loan(session_), z_keyexpr_loan(&keyexpr), z_move(payload), &opts) < 0) {
      z_keyexpr_drop(z_move(keyexpr));
      throw std::runtime_error("zenoh: z_put failed");
    }
    z_keyexpr_drop(z_move(keyexpr));
  }

  void Subscribe(std::string_view resource, SubCallback cb) {
    z_owned_keyexpr_t keyexpr{};
    size_t len = resource.size();
    if (z_keyexpr_from_substr_autocanonize(&keyexpr, resource.data(), &len) < 0) {
      throw std::runtime_error("zenoh: invalid keyexpr");
    }

    auto* pcb = new SubCallback(std::move(cb));
    z_owned_closure_sample_t clos{};
    z_closure(
        &clos,
        [](z_loaned_sample_t* sample, void* ctx) {
          auto* f = static_cast<SubCallback*>(ctx);

          auto* p = z_sample_payload(sample);
          size_t len = z_bytes_len(p);
          std::vector<uint8_t> buf(len);

          if (len) {
            auto reader = z_bytes_get_reader(p);
            size_t offset = 0;
            while (offset < len) {
              size_t n = z_bytes_reader_read(&reader, buf.data() + offset, len - offset);
              if (n == 0) {
                break;
              }
              offset += n;
            }
          }

          (*f)(std::span<const uint8_t>(buf.data(), buf.size()));
        },
        [](void* ctx) {
          auto* f = static_cast<SubCallback*>(ctx);
          delete f;
        },
        pcb);

    z_owned_subscriber_t sub{};
    if (z_declare_subscriber(z_loan(session_), &sub, z_loan(keyexpr), z_move(clos), NULL) < 0) {
      z_keyexpr_drop(z_move(keyexpr));
      throw std::runtime_error("zenoh: z_declare_subscriber failed");
    }
    z_keyexpr_drop(z_move(keyexpr));

    std::scoped_lock lk(mu_);
    subs_.emplace_back(std::move(sub));
  }

  std::vector<uint8_t> Query(std::string_view resource, Bytes req, int timeout_ms) {
    z_owned_keyexpr_t keyexpr{};
    size_t len = resource.size();
    if (z_keyexpr_from_substr_autocanonize(&keyexpr, resource.data(), &len) < 0) {
      throw std::runtime_error("zenoh: invalid keyexpr");
    }

    z_owned_bytes_t payload{};
    if (z_bytes_copy_from_buf(&payload, req.data(), req.size()) < 0) {
      z_keyexpr_drop(z_move(keyexpr));
      throw std::runtime_error("zenoh: bytes_copy_from_buf failed");
    }

    z_owned_fifo_handler_reply_t handler{};
    z_owned_closure_reply_t clos{};
    z_fifo_channel_reply_new(&clos, &handler, 1);

    z_get_options_t gopts{};
    z_get_options_default(&gopts);
    gopts.payload = z_bytes_move(&payload);
    gopts.timeout_ms = static_cast<uint64_t>(timeout_ms);
    gopts.target = Z_QUERY_TARGET_BEST_MATCHING;

    if (z_get(z_loan(session_), z_loan(keyexpr), "", z_move(clos), &gopts) < 0) {
      z_keyexpr_drop(z_move(keyexpr));
      z_fifo_handler_reply_drop(z_move(handler));
      throw std::runtime_error("zenoh: z_get failed");
    }

    z_owned_reply_t reply{};
    if (z_result_t res = z_recv(z_loan(handler), &reply); res != Z_OK) {
      z_keyexpr_drop(z_move(keyexpr));
      z_fifo_handler_reply_drop(z_move(handler));
      throw std::runtime_error("zenoh: z_recv failed: " + std::to_string(res));
    }

    if (!z_reply_is_ok(z_loan(reply))) {
      z_drop(z_move(reply));
      z_keyexpr_drop(z_move(keyexpr));
      z_fifo_handler_reply_drop(z_move(handler));
      throw std::runtime_error("zenoh: z_reply is not ok");
    }

    const auto* sample = z_reply_ok(z_loan(reply));
    auto* p = z_sample_payload(sample);
    size_t l = z_bytes_len(p);
    std::vector<uint8_t> buf(l);

    if (l) {
      auto reader = z_bytes_get_reader(p);
      size_t offset = 0;
      while (offset < l) {
        size_t n = z_bytes_reader_read(&reader, buf.data() + offset, l - offset);
        if (n == 0) {
          break;
        }
        offset += n;
      }
    }

    z_drop(z_move(reply));
    z_keyexpr_drop(z_move(keyexpr));
    z_fifo_handler_reply_drop(z_move(handler));

    return buf;
  }

  void DeclareQueryable(std::string_view resource, QueryHandler handler) {
    z_owned_keyexpr_t keyexpr{};
    size_t klen = resource.size();
    if (z_keyexpr_from_substr_autocanonize(&keyexpr, resource.data(), &klen) < 0) {
      throw std::runtime_error("zenoh: invalid keyexpr");
    }

    z_view_string_t kx_view{};
    z_keyexpr_as_view_string(z_loan(keyexpr), &kx_view);
    std::string reply_keycanon(z_string_data(z_loan(kx_view)), z_string_len(z_loan(kx_view)));

    struct Ctx {
      QueryHandler handler;
      std::string reply_keycanon;
    };

    auto* ctx = new Ctx{std::move(handler), std::move(reply_keycanon)};

    z_owned_closure_query_t clos{};
    z_closure(
        &clos,
        [](z_loaned_query_t* q, void* p) {
          auto* c = static_cast<Ctx*>(p);

          const z_loaned_bytes_t* b = z_query_payload(q);
          const size_t total = z_bytes_len(b);
          std::vector<uint8_t> in(total);
          if (total) {
            z_bytes_reader_t r = z_bytes_get_reader(b);
            size_t offset = 0;
            while (offset < total) {
              size_t n = z_bytes_reader_read(&r, in.data() + offset, total - offset);
              if (n == 0) {
                break;
              }
              offset += n;
            }
          }

          std::vector<uint8_t> out = c->handler(std::span<const uint8_t>(in.data(), in.size()));

          z_view_keyexpr_t rkey{};
          z_view_keyexpr_from_str(&rkey, c->reply_keycanon.c_str());

          z_owned_bytes_t rep{};
          if (z_bytes_copy_from_buf(&rep, out.data(), out.size()) < 0) {
            // TODO `z_bytes_copy_from_buf` 에러 발생시 처리 필요
            return;
          }
          z_query_reply(q, z_loan(rkey), z_move(rep), nullptr);
        },
        [](void* p) { delete static_cast<Ctx*>(p); }, ctx);

    z_owned_queryable_t qbl{};
    if (z_declare_queryable(z_loan(session_), &qbl, z_loan(keyexpr), z_move(clos), nullptr) < 0) {
      z_keyexpr_drop(z_move(keyexpr));
      delete ctx;
      throw std::runtime_error("zenoh: declare_queryable failed");
    }
    z_keyexpr_drop(z_move(keyexpr));

    std::scoped_lock lk(mu_);
    qbls_.emplace_back(std::move(qbl));
  }
};

ZenohBackend::ZenohBackend(ZenohConfig config) : impl_(std::make_unique<Impl>(std::move(config))) {}

ZenohBackend::~ZenohBackend() = default;

void ZenohBackend::Publish(std::string_view resource, Bytes bytes) {
  impl_->Publish(resource, bytes);
}

void ZenohBackend::Subscribe(std::string_view resource, SubCallback cb) {
  impl_->Subscribe(resource, std::move(cb));
}

std::vector<uint8_t> ZenohBackend::Query(std::string_view resource, Bytes req, int timeout_ms) {
  return impl_->Query(resource, req, timeout_ms);
}

void ZenohBackend::DeclareQueryable(std::string_view resource, QueryHandler handler) {
  impl_->DeclareQueryable(resource, std::move(handler));
}

}  // namespace rb::io