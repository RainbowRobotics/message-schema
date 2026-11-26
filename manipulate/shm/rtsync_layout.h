#ifndef RTSYNC_LAYOUT_H
#define RTSYNC_LAYOUT_H

// ═══════════════════════════════════════════════════════════════════
//  rtsync_layout.h — 파트 코어(오른팔/왼팔/몸체)와 상위 프로그램(전신제어)이
//                    공유하는 RB_RTSYNC_SHM 레이아웃 + 프로토콜 (header-only)
//
//  ※ message-schema 의 manipulate/shm/ 에 있다. 각 레포는 subtree 경로로 include 한다. (복사 금지)
//    rb_program: rb_ipc/comm_fbs/manipulate/shm/, rby2 rb_sdk: schemas/manipulate/shm/
//  ※ 외부 의존성 없음 (rb_common 등 사용 금지). 로그는 호출자가 반환 문자열로 처리.
//  ※ 레이아웃을 바꾸면 반드시 RTSYNC_VERSION 을 올릴 것.
//
//  타임라인 (1주기 파이프라인, T = 2ms, tick k 시각 = epoch + k*T)
//    k*T + 수십us   파트 i : up[i] 에 tick k 데이터(float 41개) 공개
//                           down 에서 target_tick == k 지령(float 52개) 수신
//    k*T + 500us    상위   : 3파트의 tick k 데이터 수집 → 계산
//    k*T + 1500us   상위   : 이 시각 전까지 target_tick = k+1 지령 기록 (늦으면 기록 안 함)
//    (k+1)*T        파트   : 지령 수신 ...
//
//  동기화 규칙
//    - 각 버퍼 슬롯은 seqlock: 쓰는 쪽이 seq = (k | WRITING) → 데이터 → seq = k (release)
//      읽는 쪽은 seq == k 확인 → 복사 → seq 재확인. 다르면 버림 (torn read 방지)
//    - [k & 1] 패리티 이중 버퍼 → 정상 타이밍에서는 쓰기/읽기 슬롯이 겹치지 않음
//    - 영역별 writer 는 하나 (up[i]: 파트 i, down: 상위)
//    - RT 경로 함수(publish_*/read_*/tick_of)는 syscall 없음 → Xenomai primary mode 유지
// ═══════════════════════════════════════════════════════════════════

#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <string>
#include <thread>
#include <fcntl.h>
#include <sys/file.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

namespace rb_rtsync_layout {

    // ─────────────────────────────────────────────
    //  상수
    // ─────────────────────────────────────────────
    constexpr const char* RTSYNC_SHM_NAME   = "RB_RTSYNC_SHM";
    constexpr uint32_t    RTSYNC_MAGIC      = 0x52425359;   // 'RBSY'
    constexpr uint32_t    RTSYNC_VERSION    = 7;

    constexpr int         NO_OF_SYNC_PART   = 3;            // 0: 오른팔, 1: 왼팔, 2: 몸체 (SYS_ID_NO)
    constexpr int         NO_OF_UP_DATA     = 41;           // 파트 → 상위 (파트별)
    constexpr int         NO_OF_DOWN_DATA   = 52;           // 상위 → 파트 (파트별)

    // ── 채널 내 항목 배치 (파트/상위 공용 정의) ──
    constexpr int         NO_OF_Q_SLOT      = 7;            // 실제 관절 수 (= NO_OF_JOINT). 상위 계산에 사용
    constexpr int         JOINT_SLOT        = 10;           // 관절 그룹당 슬롯 수 (관절 수 이후 슬롯은 예약, 0)
    //  파트 → 상위 (up)
    constexpr int         UP_IDX_FOLLOW     = 0;            // [0]      1: 상위 지령 따라가는 중, 0: 아님
    constexpr int         UP_IDX_Q_REF      = 1;            // [1..10]  system_q (최종 지령 각도, deg). 직접교시/서보OFF 중엔 엔코더 값
    constexpr int         UP_IDX_Q_ENC      = 11;           // [11..20] 엔코더 각도 (deg)
    constexpr int         UP_IDX_TORQUE     = 21;           // [21..30] 측정 관절 토크 (Nm)
    constexpr int         UP_IDX_EXT_REF    = 31;           // [31..40] 부가축 레퍼런스 각도 (deg)
    //  상위 → 파트 (down)
    constexpr int         DOWN_IDX_LPF      = 0;            // [0]      로우패스필터 게인 (0~1, 0: 필터 없음, 1에 가까울수록 무거운 필터)
    constexpr int         DOWN_IDX_Q        = 1;            // [1..10]  관절 지령 각도 (deg)
    constexpr int         DOWN_IDX_TORQUE   = 11;           // [11..20] 관절 지령 토크 (Nm)
    constexpr int         DOWN_IDX_FB_GAIN  = 21;           // [21..30] 피드백 게인
    constexpr int         DOWN_IDX_FF_GAIN  = 31;           // [31..40] 피드포워드 게인
    constexpr int         DOWN_IDX_EXT_Q    = 41;           // [41..50] 부가축 지령 각도 (deg)
    constexpr int         DOWN_IDX_MODE     = 51;           // [51]     지령 모드 (DOWN_MODE_*)

    //  DOWN_IDX_MODE 값 (float 로 전달, 0.5 기준으로 판정. 0 이 기본 = 일반모드)
    constexpr float       DOWN_MODE_NORMAL   = 0.0f;        // 일반모드: 관절/부가축 지령 각도만 사용. 토크·게인은 파트 기본값(모델 토크)
    constexpr float       DOWN_MODE_ADVANCED = 1.0f;        // 고급모드: 각도 + 지령 토크(모델 토크 대체) + FB/FF 게인 반영

    static_assert(NO_OF_Q_SLOT <= JOINT_SLOT, "joint count exceeds joint slot");
    static_assert(UP_IDX_EXT_REF + JOINT_SLOT <= NO_OF_UP_DATA, "up layout overflows NO_OF_UP_DATA");
    static_assert(DOWN_IDX_EXT_Q + JOINT_SLOT <= NO_OF_DOWN_DATA, "down layout overflows NO_OF_DOWN_DATA");
    static_assert(DOWN_IDX_MODE < NO_OF_DOWN_DATA, "down layout overflows NO_OF_DOWN_DATA");

    constexpr uint64_t    RTSYNC_PERIOD_NS        = 2'000'000;  // 2ms (= RT_PERIOD_MS)
    constexpr uint64_t    UPPER_WAKE_OFFSET_NS    =   500'000;  // 상위 기상: k*T + 500us
    constexpr uint64_t    UPPER_WAIT_DEADLINE_NS  = 1'000'000;  // 상위가 파트 데이터를 기다리는 한계: k*T + 1000us
    constexpr uint64_t    UPPER_WRITE_DEADLINE_NS = 1'500'000;  // 상위 지령 기록 마감: k*T + 1500us

    constexpr uint64_t    SEQ_WRITING       = 1ull << 63;   // seqlock: 기록 중 표시

    // 시계 종류: 다른 빌드가 정한 epoch 은 쓰지 않는다
    constexpr uint32_t    CLOCK_DOMAIN_XENOMAI = 1;
    constexpr uint32_t    CLOCK_DOMAIN_POSIX   = 2;

    // ─────────────────────────────────────────────
    //  레이아웃 (trivially copyable 데이터 + lock-free atomic 만, 포인터 금지)
    // ─────────────────────────────────────────────
    struct alignas(64) MonitorSlot {                // 파트 i 만 씀 (위상 모니터용)
        std::atomic<uint64_t> wake_ns;              // 최근 기상 시각
        std::atomic<int64_t>  phase_err_ns;         // 최근 위상 오차 (기상 시각 - 정렬 시각)
        std::atomic<uint64_t> tick;                 // 최근 tick, 마지막에 기록
    };

    struct alignas(64) UpSlot {                     // 파트 i 만 씀, [tick & 1]
        std::atomic<uint64_t> seq;                  // seqlock (= tick, 기록 중이면 | SEQ_WRITING)
        float                 data[NO_OF_UP_DATA];
    };

    struct alignas(64) UpChannel {
        UpSlot                slot[2];
    };

    struct alignas(64) DownSlot {                   // 상위만 씀, [target_tick & 1]
        std::atomic<uint64_t> seq;                  // seqlock (= target_tick)
        float                 data[NO_OF_SYNC_PART][NO_OF_DOWN_DATA];
    };

    struct ShmLayout {
        // --- 헤더 ---
        alignas(64) std::atomic<uint32_t> magic;    // 생성자가 헤더 기록 후 마지막에 기록
        uint32_t                          version;
        uint32_t                          size;
        uint32_t                          clock_domain;
        uint64_t                          period_ns;

        // --- 공통 epoch ---
        alignas(64) std::atomic<uint64_t> epoch_ns; // 0 = 미정, 최초 RT 진입 프로세스가 CAS 로 기록

        // --- 위상 모니터 ---
        MonitorSlot                       mon[NO_OF_SYNC_PART];

        // --- 데이터 채널 ---
        UpChannel                         up[NO_OF_SYNC_PART];   // 파트 → 상위
        DownSlot                          down[2];               // 상위 → 파트 (3파트분 한 블록)
    };

    static_assert(std::atomic<uint64_t>::is_always_lock_free, "need lock-free 64bit atomics");
    static_assert(std::atomic<uint32_t>::is_always_lock_free, "need lock-free 32bit atomics");

    // ═════════════════════════════════════════════
    //  초기화 (비RT, RT 루프 진입 전에만 호출)
    // ═════════════════════════════════════════════
    struct Handle {
        ShmLayout* shm     = nullptr;
        int        fd      = -1;        // 프로세스 수명 동안 닫지 않음 (LOCK_SH 유지 = 사용 중 표시)
        bool       created = false;     // true: 사용자가 없어서 초기화하고 새로 만듦
    };

    namespace detail {
        template<typename F>
        inline bool wait_for(F cond, int timeout_ms){
            for(int t = 0; t < timeout_ms; t += 10){
                if(cond()) return true;
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
            return cond();
        }
    }

    // 성공 시 빈 문자열, 실패 시 사유. 실패해도 호출자는 비동기로 계속 동작하면 된다.
    //  1) <name>_INIT 에 LOCK_EX  → 시작 절차 직렬화
    //  2) SHM 에 LOCK_EX|LOCK_NB 시도: 성공 = 사용 중인 프로세스 없음 → 내용 초기화 후 헤더 기록
    //                                 실패 = 누군가 사용 중 → 기존 헤더 검증 후 attach
    //  3) SHM 에 LOCK_SH 로 전환해 유지 (fd 닫지 않음, 프로세스가 죽으면 커널이 해제)
    //  ※ shm_unlink 는 하지 않음 (같은 이름으로 서로 다른 객체를 보는 split-brain 방지)
    inline std::string open_shm(const std::string& name, uint32_t clock_domain, uint64_t period_ns, Handle& h){
        const std::string init_name = name + "_INIT";
        const int init_fd = shm_open(init_name.c_str(), O_CREAT | O_RDWR, 0666);
        if(init_fd < 0) return std::string("shm_open(init lock) failed: ") + strerror(errno);

        if(!detail::wait_for([init_fd]{ return flock(init_fd, LOCK_EX | LOCK_NB) == 0; }, 3000)){
            close(init_fd);
            return "init lock timeout";
        }

        int fd = -1;
        auto fail = [&](const std::string& msg){
            if(fd >= 0) close(fd);                  // 이 fd 의 flock 도 함께 해제됨
            flock(init_fd, LOCK_UN);
            close(init_fd);
            return msg;
        };

        fd = shm_open(name.c_str(), O_CREAT | O_RDWR, 0666);
        if(fd < 0) return fail(std::string("shm_open failed: ") + strerror(errno));

        const bool alone = (flock(fd, LOCK_EX | LOCK_NB) == 0);
        if(alone){
            // 이전 실행의 잔재(epoch, 예전 레이아웃, 다른 빌드 기록)를 모두 지움. 다시 늘리면 0 으로 채워짐
            if(ftruncate(fd, 0) != 0 || ftruncate(fd, sizeof(ShmLayout)) != 0)
                return fail(std::string("ftruncate failed: ") + strerror(errno));
        }else{
            struct stat st{};
            if(fstat(fd, &st) != 0 || st.st_size < static_cast<off_t>(sizeof(ShmLayout)))
                return fail("SHM in use with different size (other process runs a different layout)");
        }

        void* p = mmap(nullptr, sizeof(ShmLayout), PROT_READ | PROT_WRITE, MAP_SHARED | MAP_POPULATE, fd, 0);
        if(p == MAP_FAILED) return fail(std::string("mmap failed: ") + strerror(errno));
        auto* shm = static_cast<ShmLayout*>(p);

        if(alone){
            shm->version      = RTSYNC_VERSION;
            shm->size         = sizeof(ShmLayout);
            shm->clock_domain = clock_domain;
            shm->period_ns    = period_ns;
            shm->magic.store(RTSYNC_MAGIC, std::memory_order_release);
        }else{
            std::string why;
            if(shm->magic.load(std::memory_order_acquire) != RTSYNC_MAGIC) why = "magic mismatch";
            else if(shm->version != RTSYNC_VERSION)     why = "version mismatch";
            else if(shm->size != sizeof(ShmLayout))     why = "size mismatch";
            else if(shm->clock_domain != clock_domain)  why = "clock domain mismatch (Xenomai/POSIX build mixed)";
            else if(shm->period_ns != period_ns)        why = "RT period mismatch";
            if(!why.empty()){
                munmap(p, sizeof(ShmLayout));
                return fail("SHM in use but incompatible: " + why);
            }
        }

        if(flock(fd, LOCK_SH) != 0){                // LOCK_EX → LOCK_SH (전환 순간의 틈은 init 락이 막음)
            munmap(p, sizeof(ShmLayout));
            return fail(std::string("flock(LOCK_SH) failed: ") + strerror(errno));
        }
        flock(init_fd, LOCK_UN);
        close(init_fd);

        h.shm     = shm;
        h.fd      = fd;
        h.created = alone;
        return "";
    }

    // epoch 결정: 없으면 now 로 CAS. epoch 이 미래(다른 시계 기준)면 false
    inline bool get_or_set_epoch(ShmLayout* shm, uint64_t now_ns, uint64_t period_ns, uint64_t& epoch_out){
        uint64_t epoch = shm->epoch_ns.load(std::memory_order_acquire);
        if(epoch == 0){
            uint64_t expected = 0;
            epoch = shm->epoch_ns.compare_exchange_strong(expected, now_ns,
                                                          std::memory_order_acq_rel,
                                                          std::memory_order_acquire) ? now_ns : expected;
        }
        // 다른 프로세스가 내 now 직후에 기록한 경쟁(1주기 이내)은 허용
        if(epoch > now_ns && epoch - now_ns > period_ns) return false;
        epoch_out = epoch;
        return true;
    }

    // 정렬된 첫 시작 시각 (최소 1주기 이상 미래)
    inline uint64_t aligned_start(uint64_t epoch, uint64_t now_ns, uint64_t period_ns){
        if(epoch > now_ns) return epoch + 2 * period_ns;
        return epoch + ((now_ns - epoch) / period_ns + 2) * period_ns;
    }

    // ═════════════════════════════════════════════
    //  RT 경로 (syscall 없음)
    // ═════════════════════════════════════════════

    // 현재 시각이 속한 tick (가장 가까운 격자점, 일찍 깬 경우도 같은 tick)
    inline uint64_t tick_of(uint64_t epoch, uint64_t now_ns, uint64_t period_ns){
        if(now_ns <= epoch) return 0;
        return (now_ns - epoch + period_ns / 2) / period_ns;
    }

    // tick k 의 시작 시각
    inline uint64_t start_of(uint64_t epoch, uint64_t k, uint64_t period_ns){
        return epoch + k * period_ns;
    }

    namespace detail {
        template<typename Slot, typename Fn>
        inline void seq_write(Slot& s, uint64_t k, Fn write_data){
            s.seq.store(k | SEQ_WRITING, std::memory_order_relaxed);
            std::atomic_thread_fence(std::memory_order_release);
            write_data();
            s.seq.store(k, std::memory_order_release);
        }

        template<typename Slot, typename Fn>
        inline bool seq_read(const Slot& s, uint64_t k, Fn read_data){
            if(s.seq.load(std::memory_order_acquire) != k) return false;
            read_data();
            std::atomic_thread_fence(std::memory_order_acquire);
            return s.seq.load(std::memory_order_relaxed) == k;
        }
    }

    // 파트 → 상위: 파트 part 가 tick k 데이터 공개 (n < NO_OF_UP_DATA 이면 나머지 0)
    inline void publish_up(ShmLayout* shm, int part, uint64_t k, const float* data, int n){
        UpSlot& s = shm->up[part].slot[k & 1];
        if(n > NO_OF_UP_DATA) n = NO_OF_UP_DATA;
        if(n < 0)             n = 0;
        detail::seq_write(s, k, [&]{
            std::memcpy(s.data, data, sizeof(float) * n);
            std::memset(s.data + n, 0, sizeof(float) * (NO_OF_UP_DATA - n));
        });
    }

    // 상위: 파트 part 의 tick k 데이터 읽기. 아직 없거나 덮어써졌으면 false
    inline bool read_up(const ShmLayout* shm, int part, uint64_t k, float out[NO_OF_UP_DATA]){
        const UpSlot& s = shm->up[part].slot[k & 1];
        return detail::seq_read(s, k, [&]{ std::memcpy(out, s.data, sizeof(s.data)); });
    }

    // 상위 → 파트: target_tick 에 적용할 3파트 지령 기록
    inline void publish_down(ShmLayout* shm, uint64_t target_tick,
                             const float data[NO_OF_SYNC_PART][NO_OF_DOWN_DATA]){
        DownSlot& s = shm->down[target_tick & 1];
        detail::seq_write(s, target_tick, [&]{ std::memcpy(s.data, data, sizeof(s.data)); });
    }

    // 파트: tick k 에 적용할 내 지령 읽기. 없으면(상위 미동작/늦음) false
    inline bool read_down(const ShmLayout* shm, int part, uint64_t k, float out[NO_OF_DOWN_DATA]){
        const DownSlot& s = shm->down[k & 1];
        return detail::seq_read(s, k, [&]{ std::memcpy(out, s.data[part], sizeof(s.data[part])); });
    }

} // namespace rb_rtsync_layout

#endif // RTSYNC_LAYOUT_H
