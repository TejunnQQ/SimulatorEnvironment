#include <mutex>
#include <memory>
#include <iomanip>
#include <utility>
#include <iostream>
#include <iterator>
#include "../include/tai_sim.h"

using namespace tai;

Accelerator::Accelerator():
        spec_reg_(NumSpecRegs, {PEGRESS, AEGRESS, MEGRESS}),
        comm_reg_(NumCommonRegs),
        cache_(nBytesOfCache),
        dram_(nBytesOfDRAM),
        tmp_(32 * 1024 * 1024),
        cu_(this),
        mpu_(this),
        lsu_(this) {
}

Accelerator::~Accelerator() = default;

int Accelerator::Run(ProgramPtr p) {
    if (p == nullptr)
        return -1;
    if (!p->Valid()) {
        std::cerr << "Program is not valid." << std::endl;
        return -1;
    }
    program_ = p;
    spec_reg_.Set(RET, program_->Size());
    paths.clear();
    for (auto i = 0; i != p->PathNum(); ++i) {
        paths.push_back({});
    }
    cu_.Run();
    cu_.Wait();

    program_ = nullptr;
    return 0;
}

CU::CU(Accelerator* acc) : sync_(std::make_shared<Sync>()) {
    acc_ = acc;
    name_ = "CU";
    sync_->stat_ = UnitStat::Idling;
    pc_ = -1;

    std::thread([this] {
        for (;;) {
            if (sync_->stat_ == UnitStat::Running) {
                for (; pc_ != acc_->program_->Size();) {
                    (acc_->program_->operator[](pc_))->kernel_(this);
                }
                std::unique_lock<std::mutex> olk(sync_->outer_mtx_);
                sync_->stat_ = UnitStat::Idling;
                olk.unlock();
                sync_->outer_cond_.notify_one();
            } else if (sync_->stat_ == UnitStat::Shutdown) {
                break;
            } else {
                std::unique_lock<std::mutex> lk(sync_->inner_mtx_);
                while (sync_->stat_ == UnitStat::Idling) {
                    sync_->inner_cond_.wait(lk);
                }
                lk.unlock();
            }
        }
    }).detach();
}

CU::~CU() {
    if (sync_) {
        sync_->stat_ = UnitStat::Shutdown;
        sync_->inner_cond_.notify_all();
    }
}

void CU::Run() {
    this->pc_ = acc_->program_->GetEntry();
    if (sync_) {
        std::unique_lock<std::mutex> lk(sync_->inner_mtx_);
        sync_->stat_ = UnitStat::Running;
        lk.unlock();
        sync_->inner_cond_.notify_one();
    }
}

void CU::Wait() {
    std::unique_lock<std::mutex> olk(sync_->outer_mtx_);
    while (sync_->stat_ == UnitStat::Running) {
        sync_->outer_cond_.wait(olk);
    }
    olk.unlock();
}

MPU::MPU(Accelerator* acc) : sync_(std::make_shared<Sync>()) {
    acc_ = acc;
    name_ = "MPU";
    pc_ = -1;
    sync_->stat_ = UnitStat::Idling;

    std::thread([this] {
        for (;;) {
            if (sync_->stat_ == UnitStat::Running) {
                bool ret = false;
                while (true) {
                    auto i = acc_->program_->operator[](pc_);
                    if (i->type_ == Type::AiInst) {
                        auto aii = reinterpret_cast<AiInst*>(i);
                        switch (i->tag_) {
                            case Tag::MatCompute: {
                                acc_->paths.at(aii->path_).insert(aii);
                                i->kernel_(this);
                                acc_->paths.at(aii->path_).erase(aii);
                                break;
                            }
                            case Tag::VecCompute: {
                                acc_->paths.at(aii->path_).insert(aii);
                                i->kernel_(this);
                                acc_->paths.at(aii->path_).erase(aii);
                                break;
                            }
                            case Tag::Load: {
                                acc_->paths.at(aii->path_).insert(aii);
                                acc_->lsu_.ExecuteRead(i);
                                pc_ += 1;
                                break;
                            }
                            case Tag::Store: {
                                acc_->paths.at(aii->path_).insert(aii);
                                acc_->lsu_.ExecuteWrite(i);
                                pc_ += 1;
                                break;
                            }
                            case Tag::Ret: {
                                ret = true;
                                break;
                            }
                                // Fence
                            default: {
                                i->kernel_(this);
                                break;
                            }
                        }
                    } else {
                        // Basic Insts
                        i->kernel_(this);
                    }
                    if (ret) {
                        std::unique_lock<std::mutex> olk(sync_->outer_mtx_);
                        sync_->stat_ = UnitStat::Idling;
                        sync_->outer_cond_.notify_one();
                        olk.unlock();
                        break;
                    }
                }
            } else if (sync_->stat_ == UnitStat::Shutdown) {
                break;
            } else {
                std::unique_lock<std::mutex> lk(sync_->inner_mtx_);
                while (sync_->stat_ == UnitStat::Idling) {
                    sync_->inner_cond_.wait(lk);
                }
                lk.unlock();
            }
        }
    }).detach();
}

MPU::~MPU() {
    if (sync_) {
        sync_->stat_ = UnitStat::Shutdown;
        sync_->inner_cond_.notify_all();
    }
}

void MPU::Run(int pc) {
    this->pc_ = pc;
    if (sync_) {
        std::unique_lock<std::mutex> lk(sync_->inner_mtx_);
        sync_->stat_ = UnitStat::Running;
        lk.unlock();
        sync_->inner_cond_.notify_one();
    }
}

void MPU::Wait() {
    std::unique_lock<std::mutex> olk(sync_->outer_mtx_);
    while (sync_->stat_ == UnitStat::Running) {
        sync_->outer_cond_.wait(olk);
    }
    olk.unlock();
}

LSU::LSU(Accelerator* acc) : sync_(std::make_shared<Sync>()) {
    acc_ = acc;
    name_ = "LSU";
    sync_->stat_ = UnitStat::Idling;
    sync_->read_done_ = true;
    sync_->write_done_ = true;
    // Read thread
    std::thread([this] {
        std::unique_lock<std::mutex> lk(sync_->rq_mtx_);
        for (;;) {
            if (!sync_->read_queue_.empty()) {
                auto current = sync_->read_queue_.front();
                sync_->read_queue_.pop();
                lk.unlock();
                sync_->read_done_ = false;
                current();
                sync_->read_done_ = true;
                lk.lock();
            } else if (sync_->stat_ == UnitStat::Shutdown) {
                break;
            } else {
                sync_->rq_cond_.wait(lk);
            }
        }
    }).detach();
    // Write thread
    std::thread([this] {
        std::unique_lock<std::mutex> lk(sync_->wq_mtx_);
        for (;;) {
            if (!sync_->write_queue_.empty()) {
                auto current = sync_->write_queue_.front();
                sync_->write_queue_.pop();
                lk.unlock();
                sync_->write_done_ = false;
                current();
                sync_->write_done_ = true;
                lk.lock();
            } else if (sync_->stat_ == UnitStat::Shutdown) {
                break;
            } else {
                sync_->wq_cond_.wait(lk);
            }
        }
    }).detach();
}

LSU::~LSU() {
    if (sync_) {
        sync_->stat_ = UnitStat::Shutdown;
        sync_->wq_cond_.notify_all();
        sync_->rq_cond_.notify_all();
    }
}

void LSU::ExecuteRead(Instruction* ri) {
    {
        std::lock_guard<std::mutex> lk(sync_->rq_mtx_);
        sync_->read_queue_.emplace([this, ri]() {
            auto tmp = reinterpret_cast<AiInst*>(ri);
            tmp->kernel_(this);
            acc_->paths.at(tmp->path_).erase(ri);
        });
    }
    sync_->rq_cond_.notify_one();
}

void LSU::ExecuteWrite(Instruction* wi) {
    {
        std::lock_guard<std::mutex> lk(sync_->wq_mtx_);
        sync_->write_queue_.emplace([this, wi]() {
            auto tmp = reinterpret_cast<AiInst*>(wi);
            tmp->kernel_(this);
            acc_->paths.at(tmp->path_).erase(wi);
        });
    }
    sync_->wq_cond_.notify_one();
}

bool LSU::Running() {
    return !((sync_->read_queue_.empty() && sync_->read_done_) &&
             (sync_->write_queue_.empty() && sync_->write_done_));
}

DRAM::DRAM(uint32_t nbytes) {
    this->nbytes = nbytes;
    data = new uint8_t[nbytes];
}

std::map<Addr, size_t> DRAM::CMA::records = {};

DRAM::~DRAM() {
    delete[] data;
}

Registers::Registers(uint32_t num, std::set<uint32_t> clears)
        : data(num, 0), clears(std::move(clears)) {
}

Registers::~Registers() = default;

uint64_t Registers::Get(uint32_t index) {
    uint64_t res = data.at(index);
    if (clears.find(index) != clears.end()) {
        data.at(index) = 0;
    }
    return res;
}

void Registers::Set(uint32_t index, uint64_t val) {
    std::lock_guard<std::mutex> lk(mtx);
    data.at(index) = val;
}