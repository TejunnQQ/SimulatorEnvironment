#ifndef TAI_SIM_TAI_SIM_H
#define TAI_SIM_TAI_SIM_H

/*
 * TAI Device Simulator
 */

#include <map>
#include <set>
#include <queue>
#include <mutex>
#include <thread>
#include <cstdint>
#include <functional>
#include <condition_variable>
#include "tai_inst.h"
#include "tai_spec.h"

namespace tai {

    class DRAM {
    public:
        explicit DRAM(uint32_t nbytes);
        ~DRAM();

        // Copy data from src to *this, cnt is not means the total bytes,
        // but the number of elem copied.
        template <typename SrcType>
        void MemCopyFrom(Addr offset, SrcType* src, size_t cnt) {
            auto dst = reinterpret_cast<ElemType*>(data) + offset;
            for (size_t i = 0; i != cnt; ++i) {
                dst[i] = static_cast<ElemType>(src[i]);
            }
        }

        // Copy data from *this to dst, cnt is...
        template <typename DstType>
        void MemCopyTo(Addr offset, DstType* dst, size_t cnt) {
            auto src = reinterpret_cast<ElemType*>(data) + offset;
            for (size_t i = 0; i != cnt; ++i) {
                dst[i] = static_cast<DstType>(src[i]);
            }
        }

        ElemType Read(Addr addr) { return *(reinterpret_cast<ElemType*>(data) + addr); }

        void Write(Addr addr, ElemType e) { *(reinterpret_cast<ElemType*>(data) + addr) = e; }

        uint8_t* Get() { return data; }

        void* Alloc(size_t nbytes) { return reinterpret_cast<void*>(data + CMA::Alloc(nbytes)); }

        void Free(void* ptr) {
            CMA::Free(reinterpret_cast<uint64_t>(ptr) - reinterpret_cast<uint64_t>(data));
        }

    private:
        struct CMA {
            static Addr Alloc(size_t nbytes) {
                if (records.empty()) {
                    records.insert({0, nbytes});
                    return 0;
                }
                Addr res = nBytesOfDRAM;
                auto iter = records.begin();
                auto next = ++records.begin();
                while (next != records.end()) {
                    if (next->first - iter->first - iter->second > nbytes) {
                        res = iter->first + iter->second;
                        records.insert({res, nbytes});
                        return res;
                    }
                    iter++;
                    next++;
                }
                if (nbytes < records.begin()->first) {
                    records.insert({0, nbytes});
                    res = 0;
                } else if (iter->first + iter->second + nbytes < nBytesOfDRAM) {
                    res = iter->first + iter->second;
                    records.insert({res, nbytes});
                }
                return res;
            }
            static void Free(Addr addr) { records.erase(addr); }
            static void Reset() { records.clear(); }
            static std::map<Addr, size_t> records;
        };
        uint32_t nbytes;
        uint8_t* data;
    };

    class Registers {
    public:
        explicit Registers(uint32_t num, std::set<uint32_t> clears = {});
        ~Registers();

        uint64_t Get(uint32_t index);
        void Set(uint32_t index, uint64_t val);

    private:
        std::vector<uint64_t> data;
        std::set<uint32_t> clears;
        std::mutex mtx;
    };

    enum class UnitStat {
        Halt,
        Idling,
        Running,
        Shutdown,
    };

    struct Accelerator;
    struct Unit {
        int pc_;
        std::string name_;
        Accelerator* acc_;
        virtual ~Unit() = default;
    };

    struct CU : Unit {
        CU(Accelerator* acc);
        ~CU() override;
        void Run();
        void Wait();

        struct Sync {
            UnitStat stat_;
            std::mutex inner_mtx_;
            std::condition_variable inner_cond_;
            std::mutex outer_mtx_;
            std::condition_variable outer_cond_;
        };

        std::shared_ptr<Sync> sync_;
    };

    struct MPU : Unit {
        MPU(Accelerator* acc);
        ~MPU() override;
        void Run(int pc);
        void Wait();

        struct Sync {
            UnitStat stat_;
            std::mutex inner_mtx_;
            std::condition_variable inner_cond_;
            std::mutex outer_mtx_;
            std::condition_variable outer_cond_;
        };
        std::shared_ptr<Sync> sync_;
    };

    struct LSU : Unit {
        LSU(Accelerator* acc);
        ~LSU();

        void ExecuteRead(Instruction* ri);
        void ExecuteWrite(Instruction* wi);
        bool Running();

        struct Sync {
            UnitStat stat_;
            bool read_done_;
            bool write_done_;
            std::mutex wq_mtx_;
            std::condition_variable wq_cond_;
            std::queue<std::function<void()>> write_queue_;
            std::mutex rq_mtx_;
            std::condition_variable rq_cond_;
            std::queue<std::function<void()>> read_queue_;
        };
        std::shared_ptr<Sync> sync_;
    };

    struct Path {
        Path(): sync_(std::make_shared<Sync>()) {}
        void insert(Instruction *i) {
            std::lock_guard<std::mutex> lck (sync_->m_);
            insts_.insert(i);
        }
        void erase(Instruction *i) {
            std::lock_guard<std::mutex> lck (sync_->m_);
            insts_.erase(i);
            if (insts_.empty()) {
                std::lock_guard<std::mutex> l(sync_->mtx_);
                sync_->cond_.notify_all();
            }
        }
        void wait() {
            std::unique_lock<std::mutex> l(sync_->mtx_);
            while (!insts_.empty()) {
                sync_->cond_.wait(l);
            }
            l.unlock();
        }
        struct Sync {
            std::mutex m_;
            std::mutex mtx_;
            std::condition_variable cond_;
        };
        std::set<Instruction*> insts_;
        std::shared_ptr<Sync> sync_;
    };

    struct Accelerator {
        Accelerator();
        ~Accelerator();

        int Run(ProgramPtr p);

        ProgramPtr program_;
        Registers comm_reg_;
        Registers spec_reg_;
        DRAM dram_;
        DRAM cache_;
        DRAM tmp_;
        CU cu_;
        MPU mpu_;
        LSU lsu_;
        std::vector<Path> paths;
    };

}  // namespace tai

#endif //TAI_SIM_TAI_SIM_H
