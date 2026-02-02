// shareddata.cpp
#define P_NAME  "SHDDATA"

#include "shareddata.h"

#include <sys/mman.h>
#include <sys/stat.h>  // For mode constants
#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <cerrno>
#include <cstdio>


namespace rb_shareddata {
    namespace {
        int shmFD_GENERAL = -1;
        bool is_creator = false;
        ST_SHARED_DATA* sharedDataPtr = nullptr;

        ST_SHARED_DATA sharedData_Local;

        std::string SHM_NAME = "RB_COBOT_SHM";
    }

    bool initialize(std::string SHM_name) {

        SHM_NAME = SHM_name;

        // 시도 1: 새로 생성
        shmFD_GENERAL = shm_open(SHM_NAME.c_str(), O_CREAT | O_EXCL | O_RDWR, 0666);
        if (shmFD_GENERAL == -1) {
            if (errno == EEXIST) {
                // 이미 존재하면 단순 오픈
                shmFD_GENERAL = shm_open(SHM_NAME.c_str(), O_RDWR, 0666);
                if (shmFD_GENERAL == -1) {
                    perror("[SHDDATA] shm_open (existing)");
                    return false;
                }
            } else {
                perror("[SHDDATA] shm_open (create)");
                return false;
            }
        } else {
            is_creator = true; // 최초 생성자
        }

        // 크기 설정
        if (ftruncate(shmFD_GENERAL, sizeof(ST_SHARED_DATA)) == -1) {
            perror("[SHDDATA] ftruncate");
            close(shmFD_GENERAL);
            shm_unlink(SHM_NAME.c_str());
            return false;
        }

        // 매핑
        sharedDataPtr = static_cast<ST_SHARED_DATA*>(
            mmap(nullptr, sizeof(ST_SHARED_DATA), PROT_READ | PROT_WRITE, MAP_SHARED, shmFD_GENERAL, 0)
        );

        if (sharedDataPtr == MAP_FAILED) {
            perror("[SHDDATA] mmap");
            close(shmFD_GENERAL);
            shm_unlink(SHM_NAME.c_str());
            return false;
        }

        // 최초 생성 시 메모리 초기화
        if (is_creator) {
            memset(sharedDataPtr, 0, sizeof(ST_SHARED_DATA));
            printf("[SHDDATA] Shared memory created and initialized.\n");
        } else {
            printf("[SHDDATA] Shared memory attached.\n");
        }

        return true;
    }

    void finalize() {
        if (sharedDataPtr != nullptr) {
            munmap(sharedDataPtr, sizeof(ST_SHARED_DATA));
            sharedDataPtr = nullptr;
        }

        if (shmFD_GENERAL != -1) {
            close(shmFD_GENERAL);
            shmFD_GENERAL = -1;
        }

        if (is_creator) {
            // 마지막 프로세스만 해제하도록 설계 가능
            shm_unlink(SHM_NAME.c_str());
            printf("[SHDDATA] Shared memory unlinked.\n");
        }
    }

    void copySharedDataToLocal() {
        if (sharedDataPtr != nullptr) {
            sharedData_Local = *sharedDataPtr;  // sharedDataPtr의 내용을 localData에 복사
        } else {
            std::cerr << "[ERROR] Shared memory not initialized or corrupted!" << std::endl;
        }
    }

    ST_SHARED_DATA* getGlobalShm() {
        return sharedDataPtr;
    }

    ST_SHARED_DATA* getLocalShm() {
        return &sharedData_Local;
    }
}
