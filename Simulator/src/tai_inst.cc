#include <iostream>
#include <algorithm>
#include <math.h>
#include <complex.h>
#include "../include/tai_inst.h"
#include "../include/tai_sim.h"

using namespace tai;

#define PI acos(-1)

Program::Program() {
    built = false;
    path_num_ = 0;
}

Program::~Program() {
    for (auto& p : insts_) {
        delete p;
    }
}

void Program::Build() {
    path_num_ = 0;

    for (size_t i = 0; i != insts_.size();) {
        switch (insts_[i]->type_) {
            case Type::Label: {
                // not possible
                i += 1;
                break;
            }
            case Type::BasicInst: {
                i += 1;
                break;
            }
            case Type::AiInst: {
                auto tmp = reinterpret_cast<AiInst*>(insts_[i]);
                path_num_ = std::max(tmp->path_ + 1, path_num_);

                if (tmp->driver_ == Drive::Data) {
                    std::vector<AiInst*> combine;
                    while (tmp->driver_ != Drive::Inst) {
                        combine.emplace_back(tmp);
                        i += 1;
                        tmp = reinterpret_cast<AiInst*>(insts_[i]);
                    }
                    combine.insert(combine.begin(), reinterpret_cast<AiInst*>(insts_[i]));
                    i += 1;
                    int forward_reg = FWD_TMP;
                    // assert combine.size() >= 2
                    for (auto iter = combine.begin() + 1; iter != combine.end(); ++iter) {
                        (*iter)->rs0_ = forward_reg;
                        (*iter)->rd_ = forward_reg;
                    }
                    combine.back()->rd_ = combine.front()->rd_;
                    combine.front()->rd_ = forward_reg;
                    for (size_t j = 0; j != combine.size(); ++j) {
                        insts_.at(i - combine.size() + j) = combine.at(j);
                    }
                } else {
                    i += 1;
                    // Drive::Inst or Drive::None
                }

                break;
            }
        }
    }
    if (labels_.find("MAIN") == labels_.end()) {
        error_msgs_.push_back("Cannot find program entry 'MAIN'.");
    }
    //std::cout << "build " << (error_msgs_.empty() ? "succeed, " : "failed, ") << insts_.size()
    //          << " instructions, " << labels_.size() << " labels, " << error_msgs_.size()
    //          << " error(s)." << std::endl;
    if (!error_msgs_.empty()) {
        for (auto& s : error_msgs_) {
            std::cerr << s << std::endl;
        }
    }
    built = true;
}

void Program::Clear() {
    built = false;
    path_num_ = 0;
    error_msgs_.clear();
    labels_.clear();
    for (auto& p : insts_) {
        delete p;
    }
    insts_.clear();
}

uint32_t Program::Size() { return insts_.size(); }

Instruction* Program::operator[](uint32_t index) { return insts_[index]; }

bool Program::Valid() { return built && error_msgs_.empty(); }

int Program::GetEntry() { return GetPC("MAIN"); }

int Program::PathNum() { return path_num_; }

Instruction* Program::CreateLabel(const std::string& l) { return new Label{l}; }

std::string Program::GetLabel(const std::string& l) { return l; }

int Program::GetPC(const std::string& l) { return labels_.at(l); }

void Program::CreateFunc(const std::string& name, std::vector<Instruction*> ivec) {
    if (labels_.find(name) != labels_.end()) {
        error_msgs_.push_back("Name " + name + " exists");
    } else {
        labels_.emplace(name, insts_.size());
    }
    bool has_return = false;
    for (auto i : ivec) {
        switch (i->type_) {
            case Type::Label: {
                auto tmp = reinterpret_cast<Label*>(i);
                if (labels_.find(tmp->text_) != labels_.end()) {
                    error_msgs_.push_back("Label " + tmp->text_ + " exists");
                } else {
                    labels_.emplace(tmp->text_, insts_.size());
                }
                delete i;
                break;
            }
            case Type::BasicInst: {
                insts_.emplace_back(i);
                break;
            }
            case Type::AiInst: {
                if (i->tag_ == Tag::Call && name != "MAIN") {
                    error_msgs_.push_back("Call can only be executed in 'MAIN'");
                }
                if (i->tag_ == Tag::Ret) {
                    has_return = true;
                }
                insts_.emplace_back(i);
                break;
            }
        }
    }
    if (!has_return) {
        error_msgs_.emplace_back("Cannot find 'RET' in function " + name);
    }
}

Instruction* Program::Movi(uint32_t rd, int64_t imm) {
    return new BasicInst{[rd, imm](Unit* c) {
        c->acc_->comm_reg_.Set(rd, imm);
        c->pc_ += 1;
    }};
}

Instruction* Program::Add(uint32_t rd, uint32_t rs0, uint32_t rs1) {
    return new BasicInst{[rd, rs0, rs1](Unit* c) {
        c->acc_->comm_reg_.Set(rd, c->acc_->comm_reg_.Get(rs0) + c->acc_->comm_reg_.Get(rs1));
        c->pc_ += 1;
    }};
}

Instruction* Program::Addi(uint32_t rd, uint32_t rs0, int imm) {
    return new BasicInst{[rd, rs0, imm](Unit* c) {
        c->acc_->comm_reg_.Set(rd, c->acc_->comm_reg_.Get(rs0) + imm);
        c->pc_ += 1;
    }};
}

Instruction* Program::Bnei(const std::string& target, uint32_t rs0, int32_t imm) {
    return new BasicInst{[target, rs0, imm](Unit* c) {
        if (static_cast<int32_t>(c->acc_->comm_reg_.Get(rs0)) != imm) {
            c->pc_ = c->acc_->program_->GetPC(target);
        } else {
            c->pc_ += 1;
        }
    }};
}

Instruction* Program::Ret() {
    return new AiInst{[](Unit* c) {
        if (c->name_ == "CU") {
            c->pc_ = c->acc_->spec_reg_.Get(RET);
            if (c->pc_ == static_cast<int32_t>(c->acc_->program_->Size())) {
                c->acc_->mpu_.Wait();
                while (c->acc_->lsu_.Running()) {
                }
            } else {
                c->acc_->spec_reg_.Set(RET, c->acc_->program_->Size());
            }
        } else if (c->name_ == "MPU") {
            c->pc_ = c->acc_->program_->Size();
        }
    },
                      Tag::Ret};
}

Instruction* Program::Call(const std::string& target, const std::string& dev, int path, int s,
                           int n) {
    return new AiInst{[target, dev, s, n](Unit* c) {
        if (dev == "MPU") {
            c->acc_->mpu_.Wait();
            for (auto i = 0, j = s; i != n; ++i, ++j) {
                c->acc_->comm_reg_.Set(i, c->acc_->comm_reg_.Get(j));
            }
            c->acc_->mpu_.Run(c->acc_->program_->GetPC(target));
            c->pc_ += 1;
        } else if (dev == "CU") {
            for (auto i = 0, j = s; i != n; ++i, ++j) {
                c->acc_->comm_reg_.Set(i, c->acc_->comm_reg_.Get(j));
            }
            c->acc_->spec_reg_.Set(RET, c->pc_ + 1);
            c->pc_ = c->acc_->program_->GetPC(target);
        }
    },
                      Tag::Call};
}

Instruction* Program::Bne(const std::string& target, uint32_t rs0, uint32_t rs1) {
    return new BasicInst{[target, rs0, rs1](Unit* c) {
        if (c->acc_->comm_reg_.Get(rs0) != c->acc_->comm_reg_.Get(rs1)) {
            c->pc_ = c->acc_->program_->GetPC(target);
        } else {
            c->pc_ += 1;
        }
    }};
}

Instruction* Program::Fence(uint32_t path) {
    return new AiInst{[path](Unit* c) {
        c->acc_->paths.at(path).wait();
        c->pc_ += 1;
    },
                      Tag::Fence};
}

Instruction* Program::Jmp(uint32_t rd, const std::string& target) {
    return new BasicInst{[rd, target](Unit* c) {
        c->acc_->comm_reg_.Set(rd, c->pc_ + 1);
        c->pc_ = c->acc_->program_->labels_.at(target);
    }};
}

Instruction* Program::Jmpr(uint32_t rd, uint32_t rs0, int offset) {

    return new BasicInst{[rd, rs0, offset](Unit* c) {
        c->acc_->comm_reg_.Set(rd, c->pc_ + 1);
        c->pc_ = c->acc_->comm_reg_.Get(rs0) + offset;
    }};
}

Instruction* Program::Beq(const std::string& target, uint32_t rs0, uint32_t rs1) {
    return new BasicInst{[target, rs0, rs1](Unit* c) {
        if (c->acc_->comm_reg_.Get(rs0) == c->acc_->comm_reg_.Get(rs1)) {
            c->pc_ = c->acc_->program_->GetPC(target);
        } else {
            c->pc_ += 1;
        }
    }};
}

Instruction* Program::Beqi(const std::string& target, uint32_t rs0, int32_t imm) {
    return new BasicInst{[target, rs0, imm](Unit* c) {
        if (static_cast<int32_t>(c->acc_->comm_reg_.Get(rs0)) == imm) {
            c->pc_ = c->acc_->program_->GetPC(target);
        } else {
            c->pc_ += 1;
        }
    }};
}

Instruction* Program::Blt(const std::string& target, uint32_t rs0, uint32_t rs1) {
    return new BasicInst{[target, rs0, rs1](Unit* c) {
        if (c->acc_->comm_reg_.Get(rs0) < c->acc_->comm_reg_.Get(rs1)) {
            c->pc_ = c->acc_->program_->GetPC(target);
        } else {
            c->pc_ += 1;
        }
    }};
}

Instruction* Program::Blti(const std::string& target, uint32_t rs0, int32_t imm) {
    return new BasicInst{[target, rs0, imm](Unit* c) {
        if (static_cast<int32_t>(c->acc_->comm_reg_.Get(rs0)) < imm) {
            c->pc_ = c->acc_->program_->GetPC(target);
        } else {
            c->pc_ += 1;
        }
    }};
}

Instruction* Program::Bnl(const std::string& target, uint32_t rs0, uint32_t rs1) {
    return new BasicInst{[target, rs0, rs1](Unit* c) {
        if (c->acc_->comm_reg_.Get(rs0) >= c->acc_->comm_reg_.Get(rs1)) {
            c->pc_ = c->acc_->program_->GetPC(target);
        } else {
            c->pc_ += 1;
        }
    }};
}

Instruction* Program::Bnli(const std::string& target, uint32_t rs0, int32_t imm) {
    return new BasicInst{[target, rs0, imm](Unit* c) {
        if (static_cast<int32_t>(c->acc_->comm_reg_.Get(rs0)) >= imm) {
            c->pc_ = c->acc_->program_->GetPC(target);
        } else {
            c->pc_ += 1;
        }
    }};
}

Instruction* Program::Mov(uint32_t rd, uint32_t rs0) {
    return new BasicInst{[rd, rs0](Unit* c) {
        c->acc_->comm_reg_.Set(rd, c->acc_->comm_reg_.Get(rs0));
        c->pc_ += 1;
    }};
}

Instruction* Program::Movid(uint32_t drd, int64_t imm) {
    return new BasicInst{[drd, imm](Unit* c) {
        c->acc_->spec_reg_.Set(drd, imm);
        c->pc_ += 1;
    }};
}

Instruction* Program::Xmovi(uint32_t rd, uint32_t rs0) {
    return new BasicInst{[rd, rs0](Unit* c) {
        c->acc_->comm_reg_.Set(rd,
                               c->acc_->dram_.Read(c->acc_->comm_reg_.Get(rs0)));
        c->pc_ += 1;
    }};
}

Instruction* Program::Xmovo(uint32_t rd, uint32_t rs0) {
    return new BasicInst{[rd, rs0](Unit* c) {
        c->acc_->dram_.Write(c->acc_->comm_reg_.Get(rd),
                             c->acc_->comm_reg_.Get(rs0));
        c->pc_ += 1;
    }};
}

Instruction* Program::Dmovi(uint32_t rd, uint32_t drs0) {
    return new BasicInst{[rd, drs0](Unit* c) {
        c->acc_->comm_reg_.Set(rd, c->acc_->spec_reg_.Get(drs0));
        c->pc_ += 1;
    }};
}

Instruction* Program::Dmovo(uint32_t drd, uint32_t rs0) {
    return new BasicInst{[drd, rs0](Unit* c) {
        c->acc_->spec_reg_.Set(drd, c->acc_->comm_reg_.Get(rs0));
        c->pc_ += 1;
    }};
}

Instruction* Program::Mul(uint32_t rd, uint32_t rs0, uint32_t rs1) {
    return new BasicInst{[rd, rs0, rs1](Unit* c) {
        c->acc_->comm_reg_.Set(rd,
                               c->acc_->comm_reg_.Get(rs0) * c->acc_->comm_reg_.Get(rs1));
        c->pc_ += 1;
    }};
}

Instruction* Program::Muli(uint32_t rd, uint32_t rs0, int imm) {
    return new BasicInst{[rd, rs0, imm](Unit* c) {
        c->acc_->comm_reg_.Set(rd,
                               c->acc_->comm_reg_.Get(rs0) * imm);
        c->pc_ += 1;
    }};
}

Instruction* Program::Slt(uint32_t rd, uint32_t rs0, uint32_t rs1) {
    return new BasicInst{[rd, rs0, rs1](Unit* c) {
        c->acc_->comm_reg_.Set(rd,
                               std::min(c->acc_->comm_reg_.Get(rs0), c->acc_->comm_reg_.Get(rs1)));
        c->pc_ += 1;
    }};
}

Instruction* Program::Slti(uint32_t rd, uint32_t rs0, int imm) {
    return new BasicInst{[rd, rs0, imm](Unit* c) {
        c->acc_->comm_reg_.Set(rd,
                               std::min(int(c->acc_->comm_reg_.Get(rs0)), imm));
        c->pc_ += 1;
    }};
}

Instruction* Program::Sgt(uint32_t rd, uint32_t rs0, uint32_t rs1) {
    return new BasicInst{[rd, rs0, rs1](Unit* c) {
        c->acc_->comm_reg_.Set(rd,
                               std::max(c->acc_->comm_reg_.Get(rs0), c->acc_->comm_reg_.Get(rs1)));
        c->pc_ += 1;
    }};
}

Instruction* Program::Sgti(uint32_t rd, uint32_t rs0, int imm) {
    return new BasicInst{[rd, rs0, imm](Unit* c) {
        c->acc_->comm_reg_.Set(rd,
                               std::max(int(c->acc_->comm_reg_.Get(rs0)), imm));
        c->pc_ += 1;
    }};
}

Instruction* Program::Or(uint32_t rd, uint32_t rs0, uint32_t rs1) {
    return new BasicInst{[rd, rs0, rs1](Unit* c) {
        c->acc_->comm_reg_.Set(rd,
                               c->acc_->comm_reg_.Get(rs0) | c->acc_->comm_reg_.Get(rs1));
        c->pc_ += 1;
    }};
}

Instruction* Program::Ori(uint32_t rd, uint32_t rs0, int imm) {
    return new BasicInst{[rd, rs0, imm](Unit* c) {
        c->acc_->comm_reg_.Set(rd,
                               c->acc_->comm_reg_.Get(rs0) | imm);
        c->pc_ += 1;
    }};
}

Instruction* Program::And(uint32_t rd, uint32_t rs0, uint32_t rs1) {
    return new BasicInst{[rd, rs0, rs1](Unit* c) {
        c->acc_->comm_reg_.Set(rd,
                               c->acc_->comm_reg_.Get(rs0) & c->acc_->comm_reg_.Get(rs1));
        c->pc_ += 1;
    }};
}

Instruction* Program::Andi(uint32_t rd, uint32_t rs0, int imm) {
    return new BasicInst{[rd, rs0, imm](Unit* c) {
        c->acc_->comm_reg_.Set(rd,
                               c->acc_->comm_reg_.Get(rs0) & imm);
        c->pc_ += 1;
    }};
}

Instruction* Program::Xor(uint32_t rd, uint32_t rs0, uint32_t rs1) {
    return new BasicInst{[rd, rs0, rs1](Unit* c) {
        c->acc_->comm_reg_.Set(rd,
                               c->acc_->comm_reg_.Get(rs0) ^ c->acc_->comm_reg_.Get(rs1));
        c->pc_ += 1;
    }};
}

Instruction* Program::Xori(uint32_t rd, uint32_t rs0, int imm) {
    return new BasicInst{[rd, rs0, imm](Unit* c) {
        c->acc_->comm_reg_.Set(rd,
                               c->acc_->comm_reg_.Get(rs0) ^ imm);
        c->pc_ += 1;
    }};
}

Instruction* Program::Srl(uint32_t rd, uint32_t rs0, uint32_t rs1) {
    return new BasicInst{[rd, rs0, rs1](Unit* c) {
        c->acc_->comm_reg_.Set(rd,
                               c->acc_->comm_reg_.Get(rs0) << c->acc_->comm_reg_.Get(rs1));
        c->pc_ += 1;
    }};
}

Instruction* Program::Srli(uint32_t rd, uint32_t rs0, int imm) {
    return new BasicInst{[rd, rs0, imm](Unit* c) {
        c->acc_->comm_reg_.Set(rd,
                               c->acc_->comm_reg_.Get(rs0) << imm);
        c->pc_ += 1;
    }};
}

Instruction* Program::Sll(uint32_t rd, uint32_t rs0, uint32_t rs1) {
    return new BasicInst{[rd, rs0, rs1](Unit* c) {
        c->acc_->comm_reg_.Set(rd,
                               c->acc_->comm_reg_.Get(rs0) >> c->acc_->comm_reg_.Get(rs1));
        c->pc_ += 1;
    }};
}

Instruction* Program::Slli(uint32_t rd, uint32_t rs0, int imm) {
    return new BasicInst{[rd, rs0, imm](Unit* c) {
        c->acc_->comm_reg_.Set(rd,
                               c->acc_->comm_reg_.Get(rs0) >> imm);
        c->pc_ += 1;
    }};
}

// 1/12
Instruction* Program::VaddI32(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs0, uint32_t rs1) {
    auto res = new AiInst{path, dri, dro, [](Unit*) {}, Tag::VecCompute};
    res->kernel_ = [res](Unit *c) {
        auto rdp = reinterpret_cast<int32_t *>(c->acc_->comm_reg_.Get(res->rd_));
        auto rp0 = reinterpret_cast<int32_t *>(c->acc_->comm_reg_.Get(res->rs0_));
        auto rp1 = reinterpret_cast<int32_t *>(c->acc_->comm_reg_.Get(res->rs1_));
        uint32_t len = c->acc_->spec_reg_.Get(VLEN);
        for (uint32_t i = 0; i < len; ++i) {
            rdp[i] = rp0[i] + rp1[i];
        }
        c->pc_ += 1;
    };
    res->rd_ = rd;
    res->rs0_ = rs0;
    res->rs1_ = rs1;
    res->name = "VADDI32";
    return res;
}
Instruction* Program::VsubI32(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs0, uint32_t rs1) {
    auto res = new AiInst{path, dri, dro, [](Unit*) {}, Tag::VecCompute};
    res->kernel_ = [res](Unit *c) {
        auto rdp = reinterpret_cast<int32_t *>(c->acc_->comm_reg_.Get(res->rd_));
        auto rp0 = reinterpret_cast<int32_t *>(c->acc_->comm_reg_.Get(res->rs0_));
        auto rp1 = reinterpret_cast<int32_t *>(c->acc_->comm_reg_.Get(res->rs1_));
        uint32_t len = c->acc_->spec_reg_.Get(VLEN);
        for (uint32_t i = 0; i < len; ++i) {
            rdp[i] = rp0[i] - rp1[i];
        }
        c->pc_ += 1;
    };
    res->rd_ = rd;
    res->rs0_ = rs0;
    res->rs1_ = rs1;
    res->name = "VSUBI32";
    return res;
}
Instruction* Program::VmulI32(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs0, uint32_t rs1) {
    auto res = new AiInst{path, dri, dro, [](Unit*) {}, Tag::VecCompute};
    res->kernel_ = [res](Unit *c) {
        auto rdp = reinterpret_cast<int32_t *>(c->acc_->comm_reg_.Get(res->rd_));
        auto rp0 = reinterpret_cast<int32_t *>(c->acc_->comm_reg_.Get(res->rs0_));
        auto rp1 = reinterpret_cast<int32_t *>(c->acc_->comm_reg_.Get(res->rs1_));
        uint32_t len = c->acc_->spec_reg_.Get(VLEN);
        for (uint32_t i = 0; i < len; ++i) {
            rdp[i] = rp0[i] * rp1[i];
        }
        c->pc_ += 1;
    };
    res->rd_ = rd;
    res->rs0_ = rs0;
    res->rs1_ = rs1;
    res->name = "VMULI32";
    return res;
}
// 2/12
Instruction* Program::VaddF32(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs0, uint32_t rs1) {
    auto res = new AiInst{path, dri, dro, [](Unit*) {}, Tag::VecCompute};
    res->kernel_ = [res](Unit *c) {
        auto rdp = reinterpret_cast<float *>(c->acc_->comm_reg_.Get(res->rd_));
        auto rp0 = reinterpret_cast<float *>(c->acc_->comm_reg_.Get(res->rs0_));
        auto rp1 = reinterpret_cast<float *>(c->acc_->comm_reg_.Get(res->rs1_));
        uint32_t len = c->acc_->spec_reg_.Get(VLEN);
        for (uint32_t i = 0; i < len; ++i) {
            rdp[i] = rp0[i] + rp1[i];
        }
        c->pc_ += 1;
    };
    res->rd_ = rd;
    res->rs0_ = rs0;
    res->rs1_ = rs1;
    res->name = "VADDF32";
    return res;
}
Instruction* Program::VsubF32(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs0, uint32_t rs1) {
    auto res = new AiInst{path, dri, dro, [](Unit*) {}, Tag::VecCompute};
    res->kernel_ = [res](Unit *c) {
        auto rdp = reinterpret_cast<float *>(c->acc_->comm_reg_.Get(res->rd_));
        auto rp0 = reinterpret_cast<float *>(c->acc_->comm_reg_.Get(res->rs0_));
        auto rp1 = reinterpret_cast<float *>(c->acc_->comm_reg_.Get(res->rs1_));
        uint32_t len = c->acc_->spec_reg_.Get(VLEN);
        for (uint32_t i = 0; i < len; ++i) {
            rdp[i] = rp0[i] - rp1[i];
        }
        c->pc_ += 1;
    };
    res->rd_ = rd;
    res->rs0_ = rs0;
    res->rs1_ = rs1;
    res->name = "VSUBF32";
    return res;
}
Instruction* Program::VmulF32(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs0, uint32_t rs1) {
    auto res = new AiInst{path, dri, dro, [](Unit*) {}, Tag::VecCompute};
    res->kernel_ = [res](Unit *c) {
        auto rdp = reinterpret_cast<float *>(c->acc_->comm_reg_.Get(res->rd_));
        auto rp0 = reinterpret_cast<float *>(c->acc_->comm_reg_.Get(res->rs0_));
        auto rp1 = reinterpret_cast<float *>(c->acc_->comm_reg_.Get(res->rs1_));
        uint32_t len = c->acc_->spec_reg_.Get(VLEN);
        for (uint32_t i = 0; i < len; ++i) {
            rdp[i] = rp0[i] * rp1[i];
        }
        c->pc_ += 1;
    };
    res->rd_ = rd;
    res->rs0_ = rs0;
    res->rs1_ = rs1;
    res->name = "VMULF32";
    return res;
}
//working 3/12
Instruction* Program::VaddF64(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs0, uint32_t rs1) {
    auto res = new AiInst{path, dri, dro, [](Unit*) {}, Tag::VecCompute};
    res->kernel_ = [res](Unit *c) {
        auto rdp = reinterpret_cast<double *>(c->acc_->comm_reg_.Get(res->rd_));
        auto rp0 = reinterpret_cast<double *>(c->acc_->comm_reg_.Get(res->rs0_));
        auto rp1 = reinterpret_cast<double *>(c->acc_->comm_reg_.Get(res->rs1_));
        uint32_t len = c->acc_->spec_reg_.Get(VLEN);
        for (uint32_t i = 0; i < len; ++i) {
            rdp[i] = rp0[i] + rp1[i];
        }
        c->pc_ += 1;
    };
    res->rd_ = rd;
    res->rs0_ = rs0;
    res->rs1_ = rs1;
    res->name = "VADDF64";
    return res;
}
Instruction* Program::VsubF64(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs0, uint32_t rs1) {
    auto res = new AiInst{path, dri, dro, [](Unit*) {}, Tag::VecCompute};
    res->kernel_ = [res](Unit *c) {
        auto rdp = reinterpret_cast<double *>(c->acc_->comm_reg_.Get(res->rd_));
        auto rp0 = reinterpret_cast<double *>(c->acc_->comm_reg_.Get(res->rs0_));
        auto rp1 = reinterpret_cast<double *>(c->acc_->comm_reg_.Get(res->rs1_));
        uint32_t len = c->acc_->spec_reg_.Get(VLEN);
        for (uint32_t i = 0; i < len; ++i) {
            rdp[i] = rp0[i] - rp1[i];
        }
        c->pc_ += 1;
    };
    res->rd_ = rd;
    res->rs0_ = rs0;
    res->rs1_ = rs1;
    res->name = "VSUBF64";
    return res;
}
Instruction* Program::VmulF64(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs0, uint32_t rs1) {
    auto res = new AiInst{path, dri, dro, [](Unit*) {}, Tag::VecCompute};
    res->kernel_ = [res](Unit *c) {
        auto rdp = reinterpret_cast<double *>(c->acc_->comm_reg_.Get(res->rd_));
        auto rp0 = reinterpret_cast<double *>(c->acc_->comm_reg_.Get(res->rs0_));
        auto rp1 = reinterpret_cast<double *>(c->acc_->comm_reg_.Get(res->rs1_));
        uint32_t len = c->acc_->spec_reg_.Get(VLEN);
        for (uint32_t i = 0; i < len; ++i) {
            rdp[i] = rp0[i] * rp1[i];
        }
        c->pc_ += 1;
    };
    res->rd_ = rd;
    res->rs0_ = rs0;
    res->rs1_ = rs1;
    res->name = "VMULF64";
    return res;
}
//working 4/12
Instruction* Program::VaddiI32(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs0, int32_t imm) {
    auto res = new AiInst{path, dri, dro, [](Unit*) {}, Tag::VecCompute};
    res->kernel_ = [res, imm](Unit *c) {
        auto rdp = reinterpret_cast<int32_t *>(c->acc_->comm_reg_.Get(res->rd_));
        auto rp0 = reinterpret_cast<int32_t *>(c->acc_->comm_reg_.Get(res->rs0_));
        uint32_t len = c->acc_->spec_reg_.Get(VLEN);
        for (uint32_t i = 0; i < len; ++i) {
            rdp[i] = rp0[i] + imm;
        }
        c->pc_ += 1;
    };
    res->rd_ = rd;
    res->rs0_ = rs0;
    // res->rs1_= imm;
    res->name = "VADDII32";
    return res;
}
Instruction* Program::VsubiI32(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs0, int32_t imm) {
    auto res = new AiInst{path, dri, dro, [](Unit*) {}, Tag::VecCompute};
    res->kernel_ = [res, imm](Unit *c) {
        auto rdp = reinterpret_cast<int32_t *>(c->acc_->comm_reg_.Get(res->rd_));
        auto rp0 = reinterpret_cast<int32_t *>(c->acc_->comm_reg_.Get(res->rs0_));
        uint32_t len = c->acc_->spec_reg_.Get(VLEN);
        for (uint32_t i = 0; i < len; ++i) {
            rdp[i] = rp0[i] - imm;
        }
        c->pc_ += 1;
    };
    res->rd_ = rd;
    res->rs0_ = rs0;
    // res->rs1_= imm;
    res->name = "VSUBII32";
    return res;
}
Instruction* Program::VmuliI32(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs0, int32_t imm) {
    auto res = new AiInst{path, dri, dro, [](Unit*) {}, Tag::VecCompute};
    res->kernel_ = [res, imm](Unit *c) {
        auto rdp = reinterpret_cast<int32_t *>(c->acc_->comm_reg_.Get(res->rd_));
        auto rp0 = reinterpret_cast<int32_t *>(c->acc_->comm_reg_.Get(res->rs0_));
        uint32_t len = c->acc_->spec_reg_.Get(VLEN);
        for (uint32_t i = 0; i < len; ++i) {
            rdp[i] = rp0[i] * imm;
        }
        c->pc_ += 1;
    };
    res->rd_ = rd;
    res->rs0_ = rs0;
    // res->rs1_= imm;
    res->name = "VMULII32";
    return res;
}
//working 5/12
Instruction* Program::VaddiF32(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs0, float imm) {
    auto res = new AiInst{path, dri, dro, [](Unit*) {}, Tag::VecCompute};
    res->kernel_ = [res, imm](Unit *c) {
        auto rdp = reinterpret_cast<float *>(c->acc_->comm_reg_.Get(res->rd_));
        auto rp0 = reinterpret_cast<float *>(c->acc_->comm_reg_.Get(res->rs0_));
        uint32_t len = c->acc_->spec_reg_.Get(VLEN);
        for (uint32_t i = 0; i < len; ++i) {
            rdp[i] = rp0[i] + imm;
        }
        c->pc_ += 1;
    };
    res->rd_ = rd;
    res->rs0_ = rs0;
    //res->rs1_ = rs1;
    res->name = "VADDIF32";
    return res;
}
Instruction* Program::VsubiF32(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs0, float imm) {
    auto res = new AiInst{path, dri, dro, [](Unit*) {}, Tag::VecCompute};
    res->kernel_ = [res, imm](Unit *c) {
        auto rdp = reinterpret_cast<float *>(c->acc_->comm_reg_.Get(res->rd_));
        auto rp0 = reinterpret_cast<float *>(c->acc_->comm_reg_.Get(res->rs0_));
        uint32_t len = c->acc_->spec_reg_.Get(VLEN);
        for (uint32_t i = 0; i < len; ++i) {
            rdp[i] = rp0[i] - imm;
        }
        c->pc_ += 1;
    };
    res->rd_ = rd;
    res->rs0_ = rs0;
    //res->rs1_ = rs1;
    res->name = "VSUBIF32";
    return res;
}
Instruction* Program::VmuliF32(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs0, float imm) {
    auto res = new AiInst{path, dri, dro, [](Unit*) {}, Tag::VecCompute};
    res->kernel_ = [res, imm](Unit *c) {
        auto rdp = reinterpret_cast<float *>(c->acc_->comm_reg_.Get(res->rd_));
        auto rp0 = reinterpret_cast<float *>(c->acc_->comm_reg_.Get(res->rs0_));
        uint32_t len = c->acc_->spec_reg_.Get(VLEN);
        for (uint32_t i = 0; i < len; ++i) {
            rdp[i] = rp0[i] * imm;
        }
        c->pc_ += 1;
    };
    res->rd_ = rd;
    res->rs0_ = rs0;
    //res->rs1_ = rs1;
    res->name = "VMULIF32";
    return res;
}
//working 6/12
Instruction* Program::VaddiF64(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs0, double imm) { 
    auto res = new AiInst{path, dri, dro, [](Unit*) {}, Tag::VecCompute};
    res->kernel_ = [res, imm](Unit *c) {
        auto rdp = reinterpret_cast<double *>(c->acc_->comm_reg_.Get(res->rd_));
        auto rp0 = reinterpret_cast<double *>(c->acc_->comm_reg_.Get(res->rs0_));
        uint32_t len = c->acc_->spec_reg_.Get(VLEN);
        for (uint32_t i = 0; i < len; ++i) {
            rdp[i] = rp0[i] + imm;
        }
        c->pc_ += 1;
    };
    res->rd_ = rd;
    res->rs0_ = rs0;
    //res->rs1_ = rs1;
    res->name = "VADDIF64";
    return res;
}
Instruction* Program::VsubiF64(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs0, double imm) { 
    auto res = new AiInst{path, dri, dro, [](Unit*) {}, Tag::VecCompute};
    res->kernel_ = [res, imm](Unit *c) {
        auto rdp = reinterpret_cast<double *>(c->acc_->comm_reg_.Get(res->rd_));
        auto rp0 = reinterpret_cast<double *>(c->acc_->comm_reg_.Get(res->rs0_));
        uint32_t len = c->acc_->spec_reg_.Get(VLEN);
        for (uint32_t i = 0; i < len; ++i) {
            rdp[i] = rp0[i] - imm;
        }
        c->pc_ += 1;
    };
    res->rd_ = rd;
    res->rs0_ = rs0;
    //res->rs1_ = rs1;
    res->name = "VSUBIF64";
    return res;
}
Instruction* Program::VmuliF64(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs0, double imm) {
    auto res = new AiInst{path, dri, dro, [](Unit*) {}, Tag::VecCompute};
    res->kernel_ = [res, imm](Unit *c) {
        auto rdp = reinterpret_cast<double *>(c->acc_->comm_reg_.Get(res->rd_));
        auto rp0 = reinterpret_cast<double *>(c->acc_->comm_reg_.Get(res->rs0_));
        uint32_t len = c->acc_->spec_reg_.Get(VLEN);
        for (uint32_t i = 0; i < len; ++i) {
            rdp[i] = rp0[i] * imm;
        }
        c->pc_ += 1;
    };
    res->rd_ = rd;
    res->rs0_ = rs0;
    //res->rs1_ = rs1;
    res->name = "VMULIF64";
    return res;
}

//working 7/12
Instruction* Program::VabsI32(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs){
    auto res = new AiInst{path, dri, dro, [](Unit*) {}, Tag::VecCompute};
    res->kernel_ = [res](Unit *c) {
        auto rdp = reinterpret_cast<int32_t *>(c->acc_->comm_reg_.Get(res->rd_));
        auto rp0 = reinterpret_cast<int32_t *>(c->acc_->comm_reg_.Get(res->rs0_));
        uint32_t len = c->acc_->spec_reg_.Get(VLEN);
        for (uint32_t i = 0; i < len; ++i) {
            rdp[i] = abs(rp0[i]);
        }
        c->pc_ += 1;
    };
    res->rd_ = rd;
    res->rs0_ = rs;
    //res->rs1_ = rs1;
    res->name = "VABSI32";
    return res;
}
Instruction* Program::VabsF32(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs){
    auto res = new AiInst{path, dri, dro, [](Unit*) {}, Tag::VecCompute};
    res->kernel_ = [res](Unit *c) {
        auto rdp = reinterpret_cast<float *>(c->acc_->comm_reg_.Get(res->rd_));
        auto rp0 = reinterpret_cast<float *>(c->acc_->comm_reg_.Get(res->rs0_));
        uint32_t len = c->acc_->spec_reg_.Get(VLEN);
        for (uint32_t i = 0; i < len; ++i) {
            rdp[i] = fabsf(rp0[i]);
        }
        c->pc_ += 1;
    };
    res->rd_ = rd;
    res->rs0_ = rs;
    //res->rs1_ = rs1;
    res->name = "VABSF32";
    return res;
}
Instruction* Program::VabsF64(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs){
    auto res = new AiInst{path, dri, dro, [](Unit*) {}, Tag::VecCompute};
    res->kernel_ = [res](Unit *c) {
        auto rdp = reinterpret_cast<double *>(c->acc_->comm_reg_.Get(res->rd_));
        auto rp0 = reinterpret_cast<double *>(c->acc_->comm_reg_.Get(res->rs0_));
        uint32_t len = c->acc_->spec_reg_.Get(VLEN);
        for (uint32_t i = 0; i < len; ++i) {
            rdp[i] = fabs(rp0[i]);
        }
        c->pc_ += 1;
    };
    res->rd_ = rd;
    res->rs0_ = rs;
    //res->rs1_ = rs1;
    res->name = "VABSF64";
    return res;
}
Instruction* Program::VabsC32(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs){
    auto res = new AiInst{path, dri, dro, [](Unit*) {}, Tag::VecCompute};
    res->kernel_ = [res](Unit *c) {
        auto rdp = reinterpret_cast<float*>(c->acc_->comm_reg_.Get(res->rd_));
        auto rp0 = reinterpret_cast<float _Complex*>(c->acc_->comm_reg_.Get(res->rs0_));
        uint32_t len = c->acc_->spec_reg_.Get(VLEN);
        for (uint32_t i = 0; i < len; ++i) {
            rdp[i] = cabsf(rp0[i]);
        }
        c->pc_ += 1;
    };
    res->rd_ = rd;
    res->rs0_ = rs;
    //res->rs1_ = rs1;
    res->name = "VABSC32";
    return res;
}
Instruction* Program::VabsC64(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs){
    auto res = new AiInst{path, dri, dro, [](Unit*) {}, Tag::VecCompute};
    res->kernel_ = [res](Unit *c) {
        auto rdp = reinterpret_cast<double*>(c->acc_->comm_reg_.Get(res->rd_));
        auto rp0 = reinterpret_cast<double _Complex*>(c->acc_->comm_reg_.Get(res->rs0_));
        uint32_t len = c->acc_->spec_reg_.Get(VLEN);
        for (uint32_t i = 0; i < len; ++i) {
            rdp[i] = cabs(rp0[i]);
        }
        c->pc_ += 1;
    };
    res->rd_ = rd;
    res->rs0_ = rs;
    //res->rs1_ = rs1;
    res->name = "VABSC64";
    return res;
}

// 8/12
Instruction* Program::VsquaI32(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs){
    auto res = new AiInst{path, dri, dro, [](Unit*){}, Tag::VecCompute};
    res->kernel_ = [res](Unit *c){
        auto rdp = reinterpret_cast<int32_t *>(c->acc_->comm_reg_.Get(res->rd_));
        auto rp0 = reinterpret_cast<int32_t *>(c->acc_->comm_reg_.Get(res->rs0_));
        uint32_t len = c->acc_->spec_reg_.Get(VLEN);
        for (uint32_t i = 0; i < len; ++i) {
            rdp[i] = pow(rp0[i],2);
        }
        c->pc_ += 1;
    };
    res->rd_ = rd;
    res->rs0_ = rs;
    res->name = "VSQUAI32";
    return res;
}
Instruction* Program::VsquaF32(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs){
    auto res = new AiInst{path, dri, dro, [](Unit*){}, Tag::VecCompute};
    res->kernel_ = [res](Unit *c){
        auto rdp = reinterpret_cast<float *>(c->acc_->comm_reg_.Get(res->rd_));
        auto rp0 = reinterpret_cast<float *>(c->acc_->comm_reg_.Get(res->rs0_));
        uint32_t len = c->acc_->spec_reg_.Get(VLEN);
        for (uint32_t i = 0; i < len; ++i) {
            rdp[i] = powf(rp0[i],2);
        }
        c->pc_ += 1;
    };
    res->rd_ = rd;
    res->rs0_ = rs;
    res->name = "VSQUAF32";
    return res;
}
Instruction* Program::VsquaF64(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs){
    auto res = new AiInst{path, dri, dro, [](Unit*){}, Tag::VecCompute};
    res->kernel_ = [res](Unit *c){
        auto rdp = reinterpret_cast<double *>(c->acc_->comm_reg_.Get(res->rd_));
        auto rp0 = reinterpret_cast<double *>(c->acc_->comm_reg_.Get(res->rs0_));
        uint32_t len = c->acc_->spec_reg_.Get(VLEN);
        for (uint32_t i = 0; i < len; ++i) {
            rdp[i] = pow(rp0[i],2);
        }
        c->pc_ += 1;
    };
    res->rd_ = rd;
    res->rs0_ = rs;
    res->name = "VSQUAF64";
    return res;
}

// 9/12
Instruction* Program::VnegI32(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs){
    auto res = new AiInst{path, dri, dro, [](Unit*){}, Tag::VecCompute};
    res->kernel_ = [res](Unit *c){
        auto rdp = reinterpret_cast<int32_t *>(c->acc_->comm_reg_.Get(res->rd_));
        auto rp0 = reinterpret_cast<int32_t *>(c->acc_->comm_reg_.Get(res->rs0_));
        uint32_t len = c->acc_->spec_reg_.Get(VLEN);
        for (uint32_t i = 0; i < len; ++i) {
            rdp[i] = -rp0[i];
        }
        c->pc_ += 1;
    };
    res->rd_ = rd;
    res->rs0_ = rs;
    res->name = "VNEGI32";
    return res;
}
Instruction* Program::VnegF32(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs){
    auto res = new AiInst{path, dri, dro, [](Unit*){}, Tag::VecCompute};
    res->kernel_ = [res](Unit *c){
        auto rdp = reinterpret_cast<float *>(c->acc_->comm_reg_.Get(res->rd_));
        auto rp0 = reinterpret_cast<float *>(c->acc_->comm_reg_.Get(res->rs0_));
        uint32_t len = c->acc_->spec_reg_.Get(VLEN);
        for (uint32_t i = 0; i < len; ++i) {
            rdp[i] = -rp0[i];
        }
        c->pc_ += 1;
    };
    res->rd_ = rd;
    res->rs0_ = rs;
    res->name = "VNEGF32";
    return res;
}
Instruction* Program::VnegF64(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs){
    auto res = new AiInst{path, dri, dro, [](Unit*){}, Tag::VecCompute};
    res->kernel_ = [res](Unit *c){
        auto rdp = reinterpret_cast<double *>(c->acc_->comm_reg_.Get(res->rd_));
        auto rp0 = reinterpret_cast<double *>(c->acc_->comm_reg_.Get(res->rs0_));
        uint32_t len = c->acc_->spec_reg_.Get(VLEN);
        for (uint32_t i = 0; i < len; ++i) {
            rdp[i] = -rp0[i];
        }
        c->pc_ += 1;
    };
    res->rd_ = rd;
    res->rs0_ = rs;
    res->name = "VNEGF64";
    return res;
}
Instruction* Program::VrecI32(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs){
    auto res = new AiInst{path, dri, dro, [](Unit*){}, Tag::VecCompute};
    res->kernel_ = [res](Unit *c){
        auto rdp = reinterpret_cast<double *>(c->acc_->comm_reg_.Get(res->rd_));
        auto rp0 = reinterpret_cast<int32_t *>(c->acc_->comm_reg_.Get(res->rs0_));
        uint32_t len = c->acc_->spec_reg_.Get(VLEN);
        for (uint32_t i = 0; i < len; ++i) {
            rdp[i] = 1.0 / rp0[i];
        }
        c->pc_ += 1;
    };
    res->rd_ = rd;
    res->rs0_ = rs;
    res->name = "VRECI32";
    return res;
}
Instruction* Program::VrecF32(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs){
    auto res = new AiInst{path, dri, dro, [](Unit*){}, Tag::VecCompute};
    res->kernel_ = [res](Unit *c){
        auto rdp = reinterpret_cast<float *>(c->acc_->comm_reg_.Get(res->rd_));
        auto rp0 = reinterpret_cast<float *>(c->acc_->comm_reg_.Get(res->rs0_));
        uint32_t len = c->acc_->spec_reg_.Get(VLEN);
        for (uint32_t i = 0; i < len; ++i) {
            rdp[i] = 1 / rp0[i];
        }
        c->pc_ += 1;
    };
    res->rd_ = rd;
    res->rs0_ = rs;
    res->name = "VRECF32";
    return res;
}
Instruction* Program::VrecF64(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs){
    auto res = new AiInst{path, dri, dro, [](Unit*){}, Tag::VecCompute};
    res->kernel_ = [res](Unit *c){
        auto rdp = reinterpret_cast<double *>(c->acc_->comm_reg_.Get(res->rd_));
        auto rp0 = reinterpret_cast<double *>(c->acc_->comm_reg_.Get(res->rs0_));
        uint32_t len = c->acc_->spec_reg_.Get(VLEN);
        for (uint32_t i = 0; i < len; ++i) {
            rdp[i] = 1 / rp0[i];
        }
        c->pc_ += 1;
    };
    res->rd_ = rd;
    res->rs0_ = rs;
    res->name = "VRECF64";
    return res;
}
// 10/12
Instruction* Program::VexpI32(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs){
    auto res = new AiInst{path, dri, dro, [](Unit*){}, Tag::VecCompute};
    res->kernel_ = [res](Unit *c){
        auto rdp = reinterpret_cast<int32_t *>(c->acc_->comm_reg_.Get(res->rd_));
        auto rp0 = reinterpret_cast<int32_t *>(c->acc_->comm_reg_.Get(res->rs0_));
        uint32_t len = c->acc_->spec_reg_.Get(VLEN);
        for (uint32_t i = 0; i < len; ++i) {
            rdp[i] = exp(rp0[i]);
        }
        c->pc_ += 1;
    };
    res->rd_ = rd;
    res->rs0_ = rs;
    res->name = "VEXPI32";
    return res;
}
Instruction* Program::VexpF32(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs){
    auto res = new AiInst{path, dri, dro, [](Unit*){}, Tag::VecCompute};
    res->kernel_ = [res](Unit *c){
        auto rdp = reinterpret_cast<float *>(c->acc_->comm_reg_.Get(res->rd_));
        auto rp0 = reinterpret_cast<float *>(c->acc_->comm_reg_.Get(res->rs0_));
        uint32_t len = c->acc_->spec_reg_.Get(VLEN);
        for (uint32_t i = 0; i < len; ++i) {
            rdp[i] = expf(rp0[i]);
        }
        c->pc_ += 1;
    };
    res->rd_ = rd;
    res->rs0_ = rs;
    res->name = "VEXPF32";
    return res;
}
Instruction* Program::VexpF64(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs){
    auto res = new AiInst{path, dri, dro, [](Unit*){}, Tag::VecCompute};
    res->kernel_ = [res](Unit *c){
        auto rdp = reinterpret_cast<double *>(c->acc_->comm_reg_.Get(res->rd_));
        auto rp0 = reinterpret_cast<double *>(c->acc_->comm_reg_.Get(res->rs0_));
        uint32_t len = c->acc_->spec_reg_.Get(VLEN);
        for (uint32_t i = 0; i < len; ++i) {
            rdp[i] = exp(rp0[i]);
        }
        c->pc_ += 1;
    };
    res->rd_ = rd;
    res->rs0_ = rs;
    res->name = "VEXPF64";
    return res;
}

// 11/12
Instruction* Program::Vlog10I32(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs){
    auto res = new AiInst{path, dri, dro, [](Unit*){}, Tag::VecCompute};
    res->kernel_ = [res](Unit *c){
        auto rdp = reinterpret_cast<double *>(c->acc_->comm_reg_.Get(res->rd_));
        auto rp0 = reinterpret_cast<int32_t *>(c->acc_->comm_reg_.Get(res->rs0_));
        uint32_t len = c->acc_->spec_reg_.Get(VLEN);
        for (uint32_t i = 0; i < len; ++i) {
            rdp[i] = log10(rp0[i]);
        }
        c->pc_ += 1;
    };
    res->rd_ = rd;
    res->rs0_ = rs;
    res->name = "VLOG10I32";
    return res;
}
Instruction* Program::Vlog10F32(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs){
    auto res = new AiInst{path, dri, dro, [](Unit*){}, Tag::VecCompute};
    res->kernel_ = [res](Unit *c){
        auto rdp = reinterpret_cast<double *>(c->acc_->comm_reg_.Get(res->rd_));
        auto rp0 = reinterpret_cast<float *>(c->acc_->comm_reg_.Get(res->rs0_));
        uint32_t len = c->acc_->spec_reg_.Get(VLEN);
        for (uint32_t i = 0; i < len; ++i) {
            rdp[i] = log10(rp0[i]);
        }
        c->pc_ += 1;
    };
    res->rd_ = rd;
    res->rs0_ = rs;
    res->name = "VLOG10F32";
    return res;
}
Instruction* Program::Vlog10F64(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs){
    auto res = new AiInst{path, dri, dro, [](Unit*){}, Tag::VecCompute};
    res->kernel_ = [res](Unit *c){
        auto rdp = reinterpret_cast<double *>(c->acc_->comm_reg_.Get(res->rd_));
        auto rp0 = reinterpret_cast<double *>(c->acc_->comm_reg_.Get(res->rs0_));
        uint32_t len = c->acc_->spec_reg_.Get(VLEN);
        for (uint32_t i = 0; i < len; ++i) {
            rdp[i] = log10(rp0[i]);
        }
        c->pc_ += 1;
    };
    res->rd_ = rd;
    res->rs0_ = rs;
    res->name = "VLOG10F64";
    return res;
}

// 12/12
Instruction* Program::VconjC32(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs) {
    auto res = new AiInst{path, dri, dro, [](Unit*) {}, Tag::VecCompute};
    res->kernel_ = [res](Unit *c) {
        auto rdp = reinterpret_cast<float *>(c->acc_->comm_reg_.Get(res->rd_));
        auto rp0 = reinterpret_cast<float *>(c->acc_->comm_reg_.Get(res->rs0_));
        uint32_t len = c->acc_->spec_reg_.Get(VLEN);
        for (uint32_t i = 0; i < len; ++i) {
            rdp[2*i] = rp0[2*i];
            rdp[2*i+1] = -rp0[2*i+1];
        }
        c->pc_ += 1;
    };
    res->rd_ = rd;
    res->rs0_ = rs;
    res->name = "VCONJC32";
    return res;
}

Instruction* Program::VconjC64(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs) {
    auto res = new AiInst{path, dri, dro, [](Unit*) {}, Tag::VecCompute};
    res->kernel_ = [res](Unit *c) {
        auto rdp = reinterpret_cast<double *>(c->acc_->comm_reg_.Get(res->rd_));
        auto rp0 = reinterpret_cast<double *>(c->acc_->comm_reg_.Get(res->rs0_));
        uint32_t len = c->acc_->spec_reg_.Get(VLEN);
        for (uint32_t i = 0; i < len; ++i) {
            rdp[2*i] = rp0[2*i];
            rdp[2*i+1] = -rp0[2*i+1];
        }
        c->pc_ += 1;
    };
    res->rd_ = rd;
    res->rs0_ = rs;
    res->name = "VCONJC64";
    return res;
}

// 1/3
Instruction* Program::VsumI32(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs){
    auto res = new AiInst{path, dri, dro, [](Unit*){}, Tag::VecCompute};
    res->kernel_ = [res](Unit *c){
        auto rdp = reinterpret_cast<int32_t *>(c->acc_->comm_reg_.Get(res->rd_));
        auto rp0 = reinterpret_cast<int32_t *>(c->acc_->comm_reg_.Get(res->rs0_));
        uint32_t len = c->acc_->spec_reg_.Get(VLEN);
        rdp[0] = 0;
        for (uint32_t i = 0; i < len; ++i) {
            rdp[0] += rp0[i];
        }
        c->pc_ += 1;
    };
    res->rd_ = rd;
    res->rs0_ = rs;
    res->name = "SUMI32";
    return res;
}
Instruction* Program::VsumF32(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs){
    auto res = new AiInst{path, dri, dro, [](Unit*){}, Tag::VecCompute};
    res->kernel_ = [res](Unit *c){
        auto rdp = reinterpret_cast<float *>(c->acc_->comm_reg_.Get(res->rd_));
        auto rp0 = reinterpret_cast<float *>(c->acc_->comm_reg_.Get(res->rs0_));
        uint32_t len = c->acc_->spec_reg_.Get(VLEN);
        rdp[0] = 0;
        for (uint32_t i = 0; i < len; ++i) {
            rdp[0] += rp0[i];
        }
        c->pc_ += 1;
    };
    res->rd_ = rd;
    res->rs0_ = rs;
    res->name = "SUMF32";
    return res;
}
Instruction* Program::VsumF64(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs){
    auto res = new AiInst{path, dri, dro, [](Unit*){}, Tag::VecCompute};
    res->kernel_ = [res](Unit *c){
        auto rdp = reinterpret_cast<double *>(c->acc_->comm_reg_.Get(res->rd_));
        auto rp0 = reinterpret_cast<double *>(c->acc_->comm_reg_.Get(res->rs0_));
        uint32_t len = c->acc_->spec_reg_.Get(VLEN);
        rdp[0] = 0;
        for (uint32_t i = 0; i < len; ++i) {
            rdp[0] += rp0[i];
        }
        c->pc_ += 1;
    };
    res->rd_ = rd;
    res->rs0_ = rs;
    res->name = "SUMF64";
    return res;
}

// 2/3
Instruction* Program::VmaxI32(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs){
    auto res = new AiInst{path, dri, dro, [](Unit*){}, Tag::VecCompute};
    res->kernel_ = [res](Unit *c){
        auto rdp = reinterpret_cast<int32_t *>(c->acc_->comm_reg_.Get(res->rd_));
        auto rp0 = reinterpret_cast<int32_t *>(c->acc_->comm_reg_.Get(res->rs0_));
        uint32_t len = c->acc_->spec_reg_.Get(VLEN);
        int32_t max = rp0[0];
        for (uint32_t i = 0; i < len; ++i) {
            if (rp0[i]>max) max = rp0[i];
        }
        rdp[0] = max;
        c->pc_ += 1;
    };
    res->rd_ = rd;
    res->rs0_ = rs;
    res->name = "MAXI32";
    return res;
}
Instruction* Program::VmaxF32(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs){
    auto res = new AiInst{path, dri, dro, [](Unit*){}, Tag::VecCompute};
    res->kernel_ = [res](Unit *c){
        auto rdp = reinterpret_cast<float *>(c->acc_->comm_reg_.Get(res->rd_));
        auto rp0 = reinterpret_cast<float *>(c->acc_->comm_reg_.Get(res->rs0_));
        uint32_t len = c->acc_->spec_reg_.Get(VLEN);
        float max = rp0[0];
        for (uint32_t i = 0; i < len; ++i) {
            if (rp0[i]>max) max = rp0[i];
        }
        rdp[0] = max;
        c->pc_ += 1;
    };
    res->rd_ = rd;
    res->rs0_ = rs;
    res->name = "MAXF32";
    return res;
}
Instruction* Program::VmaxF64(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs){
    auto res = new AiInst{path, dri, dro, [](Unit*){}, Tag::VecCompute};
    res->kernel_ = [res](Unit *c){
        auto rdp = reinterpret_cast<double *>(c->acc_->comm_reg_.Get(res->rd_));
        auto rp0 = reinterpret_cast<double *>(c->acc_->comm_reg_.Get(res->rs0_));
        uint32_t len = c->acc_->spec_reg_.Get(VLEN);
        double max = rp0[0];
        for (uint32_t i = 0; i < len; ++i) {
            if (rp0[i]>max) max = rp0[i];
        }
        rdp[0] = max;
        c->pc_ += 1;
    };
    res->rd_ = rd;
    res->rs0_ = rs;
    res->name = "MAXF64";
    return res;
}

// 3/3
Instruction* Program::VminI32(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs){
    auto res = new AiInst{path, dri, dro, [](Unit*){}, Tag::VecCompute};
    res->kernel_ = [res](Unit *c){
        auto rdp = reinterpret_cast<int32_t *>(c->acc_->comm_reg_.Get(res->rd_));
        auto rp0 = reinterpret_cast<int32_t *>(c->acc_->comm_reg_.Get(res->rs0_));
        uint32_t len = c->acc_->spec_reg_.Get(VLEN);
        int32_t min = rp0[0];
        for (uint32_t i = 0; i < len; ++i) {
            if (rp0[i]<min) min = rp0[i];
        }
        rdp[0] = min;
        c->pc_ += 1;
    };
    res->rd_ = rd;
    res->rs0_ = rs;
    res->name = "MINI32";
    return res;
}
Instruction* Program::VminF32(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs){
    auto res = new AiInst{path, dri, dro, [](Unit*){}, Tag::VecCompute};
    res->kernel_ = [res](Unit *c){
        auto rdp = reinterpret_cast<float *>(c->acc_->comm_reg_.Get(res->rd_));
        auto rp0 = reinterpret_cast<float *>(c->acc_->comm_reg_.Get(res->rs0_));
        uint32_t len = c->acc_->spec_reg_.Get(VLEN);
        float min = rp0[0];
        for (uint32_t i = 0; i < len; ++i) {
            if (rp0[i]<min) min = rp0[i];
        }
        rdp[0] = min;
        c->pc_ += 1;
    };
    res->rd_ = rd;
    res->rs0_ = rs;
    res->name = "MINF32";
    return res;
}
Instruction* Program::VminF64(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs){
    auto res = new AiInst{path, dri, dro, [](Unit*){}, Tag::VecCompute};
    res->kernel_ = [res](Unit *c){
        auto rdp = reinterpret_cast<double *>(c->acc_->comm_reg_.Get(res->rd_));
        auto rp0 = reinterpret_cast<double *>(c->acc_->comm_reg_.Get(res->rs0_));
        uint32_t len = c->acc_->spec_reg_.Get(VLEN);
        double min = rp0[0];
        for (uint32_t i = 0; i < len; ++i) {
            if (rp0[i]<min) min = rp0[i];
        }
        rdp[0] = min;
        c->pc_ += 1;
    };
    res->rd_ = rd;
    res->rs0_ = rs;
    res->name = "MINF64";
    return res;
}

// 1/2 transpose (ndim, xsize, ysize, zsize)
Instruction* Program::TransposeI32(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs){
    auto res = new AiInst{path, dri, dro, [](Unit*){}, Tag::VecCompute};
    res->kernel_ = [res](Unit *c){
        auto rdp = reinterpret_cast<int32_t *>(c->acc_->comm_reg_.Get(res->rd_));
        auto rp0 = reinterpret_cast<int32_t *>(c->acc_->comm_reg_.Get(res->rs0_));
        uint32_t ndim = c->acc_->spec_reg_.Get(NDIM);
        uint32_t x_size = c->acc_->spec_reg_.Get(X_SIZE);
        uint32_t y_size = c->acc_->spec_reg_.Get(Y_SIZE);
        uint32_t z_size = c->acc_->spec_reg_.Get(Z_SIZE);
        if (ndim == 3){
            for (uint32_t i = 0; i < x_size; ++i) {
                for (uint32_t j =0; j < y_size; ++j) {
                    for (uint32_t k=0; k < z_size; ++k){
                        rdp[i*y_size*z_size + k*y_size + j] = rp0[i*y_size*z_size + j*z_size + k];
                    }
                }
            }
        } else if(ndim == 2) {
            for (uint32_t i = 0; i < x_size; ++i) {
                for (uint32_t j =0; j < y_size; ++j) {
                    rdp[i + j*x_size] = rp0[i*y_size + j];
                }
            }
        }
        c->pc_ += 1;
    };
    res->rd_ = rd;
    res->rs0_ = rs;
    res->name = "TRANSPOSEI32";
    return res;
}
Instruction* Program::TransposeF32(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs){
    auto res = new AiInst{path, dri, dro, [](Unit*){}, Tag::VecCompute};
    res->kernel_ = [res](Unit *c){
        auto rdp = reinterpret_cast<float *>(c->acc_->comm_reg_.Get(res->rd_));
        auto rp0 = reinterpret_cast<float *>(c->acc_->comm_reg_.Get(res->rs0_));
        uint32_t ndim = c->acc_->spec_reg_.Get(NDIM);
        uint32_t x_size = c->acc_->spec_reg_.Get(X_SIZE);
        uint32_t y_size = c->acc_->spec_reg_.Get(Y_SIZE);
        uint32_t z_size = c->acc_->spec_reg_.Get(Z_SIZE);
        if (ndim == 3){
            for (uint32_t i = 0; i < x_size; ++i) {
                for (uint32_t j =0; j < y_size; ++j) {
                    for (uint32_t k=0; k < z_size; ++k){
                        rdp[i*y_size*z_size + k*y_size + j] = rp0[i*y_size*z_size + j*z_size + k];
                    }
                }
            }
        } else if(ndim == 2) {
            for (uint32_t i = 0; i < x_size; ++i) {
                for (uint32_t j =0; j < y_size; ++j) {
                    rdp[i + j*x_size] = rp0[i*y_size + j];
                }
            }
        }
        c->pc_ += 1;
    };
    res->rd_ = rd;
    res->rs0_ = rs;
    res->name = "TRANSPOSEF32";
    return res;
}
Instruction* Program::TransposeF64(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs){
    auto res = new AiInst{path, dri, dro, [](Unit*){}, Tag::VecCompute};
    res->kernel_ = [res](Unit *c){
        auto rdp = reinterpret_cast<double *>(c->acc_->comm_reg_.Get(res->rd_));
        auto rp0 = reinterpret_cast<double *>(c->acc_->comm_reg_.Get(res->rs0_));
        uint32_t ndim = c->acc_->spec_reg_.Get(NDIM);
        uint32_t x_size = c->acc_->spec_reg_.Get(X_SIZE);
        uint32_t y_size = c->acc_->spec_reg_.Get(Y_SIZE);
        uint32_t z_size = c->acc_->spec_reg_.Get(Z_SIZE);
        if (ndim == 3){
            for (uint32_t i = 0; i < x_size; ++i) {
                for (uint32_t j =0; j < y_size; ++j) {
                    for (uint32_t k=0; k < z_size; ++k){
                        rdp[i*y_size*z_size + k*y_size + j] = rp0[i*y_size*z_size + j*z_size + k];
                    }
                }
            }
        } else if(ndim == 2) {
            for (uint32_t i = 0; i < x_size; ++i) {
                for (uint32_t j =0; j < y_size; ++j) {
                    rdp[i + j*x_size] = rp0[i*y_size + j];
                }
            }
        }
        c->pc_ += 1;
    };
    res->rd_ = rd;
    res->rs0_ = rs;
    res->name = "TRANSPOSEF64";
    return res;
}

// 2/2 permute (ndim, xsize, ysize, zsize, xaxis, yaxis, zaxis)
Instruction* Program::PermuteI32(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs) {
    auto res = new AiInst{path, dri, dro, [](Unit*){}, Tag::VecCompute};
    res->kernel_ = [res](Unit *c){
        auto rdp = reinterpret_cast<int32_t *>(c->acc_->comm_reg_.Get(res->rd_));
        auto rp0 = reinterpret_cast<int32_t *>(c->acc_->comm_reg_.Get(res->rs0_));
        uint32_t ndim = c->acc_->spec_reg_.Get(NDIM);
        uint32_t x_size = c->acc_->spec_reg_.Get(X_SIZE);
        uint32_t y_size = c->acc_->spec_reg_.Get(Y_SIZE);
        uint32_t z_size = c->acc_->spec_reg_.Get(Z_SIZE);
        uint32_t x_axis = c->acc_->spec_reg_.Get(X_AXIS);
        uint32_t y_axis = c->acc_->spec_reg_.Get(Y_AXIS);
        uint32_t z_axis = c->acc_->spec_reg_.Get(Z_AXIS);
        if (ndim == 3){
            int32_t i_stride = (x_axis<y_axis? y_size : 1) * (x_axis<z_axis? z_size : 1);
            int32_t j_stride = (y_axis<x_axis? x_size : 1) * (y_axis<z_axis? z_size : 1);
            int32_t k_stride = (z_axis<x_axis? x_size : 1) * (z_axis<y_axis? y_size : 1);
            
            for (uint32_t i = 0; i < x_size; ++i) {
                for (uint32_t j = 0; j < y_size; ++j) {
                    for (uint32_t k = 0; k < z_size; ++k) {
                        rdp[i*i_stride + j*j_stride + k*k_stride] = rp0[i*y_size*z_size + j*z_size + k];
                    }
                }
            }
        } else if (ndim == 2) {
            for (uint32_t i = 0; i < x_size; ++i) {
                for (uint32_t j =0; j < y_size; ++j) {
                    rdp[i + j*x_size] = rp0[i*y_size + j];
                }
            }
        }
        c->pc_ += 1;
    };
    res->rd_ = rd;
    res->rs0_ = rs;
    res->name = "PERMUTEI32";
    return res;
}
Instruction* Program::PermuteF32(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs) {
    auto res = new AiInst{path, dri, dro, [](Unit*){}, Tag::VecCompute};
    res->kernel_ = [res](Unit *c){
        auto rdp = reinterpret_cast<float *>(c->acc_->comm_reg_.Get(res->rd_));
        auto rp0 = reinterpret_cast<float *>(c->acc_->comm_reg_.Get(res->rs0_));
        uint32_t ndim = c->acc_->spec_reg_.Get(NDIM);
        uint32_t x_size = c->acc_->spec_reg_.Get(X_SIZE);
        uint32_t y_size = c->acc_->spec_reg_.Get(Y_SIZE);
        uint32_t z_size = c->acc_->spec_reg_.Get(Z_SIZE);
        uint32_t x_axis = c->acc_->spec_reg_.Get(X_AXIS);
        uint32_t y_axis = c->acc_->spec_reg_.Get(Y_AXIS);
        uint32_t z_axis = c->acc_->spec_reg_.Get(Z_AXIS);
        if (ndim == 3){
            int32_t i_stride = (x_axis<y_axis? y_size : 1) * (x_axis<z_axis? z_size : 1);
            int32_t j_stride = (y_axis<x_axis? x_size : 1) * (y_axis<z_axis? z_size : 1);
            int32_t k_stride = (z_axis<x_axis? x_size : 1) * (z_axis<y_axis? y_size : 1);
            
            for (uint32_t i = 0; i < x_size; ++i) {
                for (uint32_t j =0; j < y_size; ++j) {
                    for (uint32_t k=0; k < z_size; ++k) {
                        rdp[i*i_stride + j*j_stride + k*k_stride] = rp0[i*y_size*z_size + j*z_size + k];
                    }
                }
            }
        } else if (ndim == 2) {
            for (uint32_t i = 0; i < x_size; ++i) {
                for (uint32_t j =0; j < y_size; ++j) {
                    rdp[i + j*x_size] = rp0[i*y_size + j];
                }
            }
        }
        c->pc_ += 1;
    };
    res->rd_ = rd;
    res->rs0_ = rs;
    res->name = "PERMUTEF32";
    return res;
}
Instruction* Program::PermuteF64(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs) {
    auto res = new AiInst{path, dri, dro, [](Unit*){}, Tag::VecCompute};
    res->kernel_ = [res](Unit *c){
        auto rdp = reinterpret_cast<double *>(c->acc_->comm_reg_.Get(res->rd_));
        auto rp0 = reinterpret_cast<double *>(c->acc_->comm_reg_.Get(res->rs0_));
        uint32_t ndim = c->acc_->spec_reg_.Get(NDIM);
        uint32_t x_size = c->acc_->spec_reg_.Get(X_SIZE);
        uint32_t y_size = c->acc_->spec_reg_.Get(Y_SIZE);
        uint32_t z_size = c->acc_->spec_reg_.Get(Z_SIZE);
        uint32_t x_axis = c->acc_->spec_reg_.Get(X_AXIS);
        uint32_t y_axis = c->acc_->spec_reg_.Get(Y_AXIS);
        uint32_t z_axis = c->acc_->spec_reg_.Get(Z_AXIS);
        if (ndim == 3){
            int32_t i_stride = (x_axis<y_axis? y_size : 1) * (x_axis<z_axis? z_size : 1);
            int32_t j_stride = (y_axis<x_axis? x_size : 1) * (y_axis<z_axis? z_size : 1);
            int32_t k_stride = (z_axis<x_axis? x_size : 1) * (z_axis<y_axis? y_size : 1);
            
            for (uint32_t i = 0; i < x_size; ++i) {
                for (uint32_t j =0; j < y_size; ++j) {
                    for (uint32_t k=0; k < z_size; ++k) {
                        rdp[i*i_stride + j*j_stride + k*k_stride] = rp0[i*y_size*z_size + j*z_size + k];
                    }
                }
            }
        } else if (ndim == 2) {
            for (uint32_t i = 0; i < x_size; ++i) {
                for (uint32_t j =0; j < y_size; ++j) {
                    rdp[i + j*x_size] = rp0[i*y_size + j];
                }
            }
        }
        c->pc_ += 1;
    };
    res->rd_ = rd;
    res->rs0_ = rs;
    res->name = "PERMUTEF64";
    return res;
}

Instruction* Program::GemmI32(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs0, uint32_t rs1) {
    auto res = new AiInst{path, dri, dro, [](Unit*) {}, Tag::VecCompute};
    res->kernel_ = [res](Unit *c) {
        auto rdp = reinterpret_cast<int32_t *>(c->acc_->comm_reg_.Get(res->rd_));
        auto rp0 = reinterpret_cast<int32_t *>(c->acc_->comm_reg_.Get(res->rs0_));
        auto rp1 = reinterpret_cast<int32_t *>(c->acc_->comm_reg_.Get(res->rs1_));
        uint32_t m = c->acc_->spec_reg_.Get(X_SIZE);
        uint32_t p = c->acc_->spec_reg_.Get(Y_SIZE);
        uint32_t n = c->acc_->spec_reg_.Get(Z_SIZE);
        for (int i = 0; i < m; ++i) {
            for (int j = 0; j < n; ++j) {
                rdp[i*n+j] = 0;
            }
        }
        for (int i = 0; i < m; ++i) {
            for (int l = 0; l < p; ++l) {
                for (int j = 0; j < n; ++j) {
                    rdp[i*n+j] += rp0[i*p+l]*rp1[l*n+j];
                }
            }
        }
        c->pc_ += 1;
    };
    res->rd_ = rd;
    res->rs0_ = rs0;
    res->rs1_ = rs1;
    res->name = "GEMM.I32";
    return res;
}
Instruction* Program::GemmF32(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs0, uint32_t rs1) {
    auto res = new AiInst{path, dri, dro, [](Unit*) {}, Tag::VecCompute};
    res->kernel_ = [res](Unit *c) {
        auto rdp = reinterpret_cast<float *>(c->acc_->comm_reg_.Get(res->rd_));
        auto rp0 = reinterpret_cast<float *>(c->acc_->comm_reg_.Get(res->rs0_));
        auto rp1 = reinterpret_cast<float *>(c->acc_->comm_reg_.Get(res->rs1_));
        uint32_t m = c->acc_->spec_reg_.Get(X_SIZE);
        uint32_t p = c->acc_->spec_reg_.Get(Y_SIZE);
        uint32_t n = c->acc_->spec_reg_.Get(Z_SIZE);
        for (int i = 0; i < m; ++i) {
            for (int j = 0; j < n; ++j) {
                rdp[i*n+j] = 0;
            }
        }
        for (int i = 0; i < m; ++i) {
            for (int l = 0; l < p; ++l) {
                for (int j = 0; j < n; ++j) {
                    rdp[i*n+j] += rp0[i*p+l]*rp1[l*n+j];
                }
            }
        }
        c->pc_ += 1;
    };
    res->rd_ = rd;
    res->rs0_ = rs0;
    res->rs1_ = rs1;
    res->name = "GEMM.F32";
    return res;
}
Instruction* Program::GemmF64(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs0, uint32_t rs1) {
    auto res = new AiInst{path, dri, dro, [](Unit*) {}, Tag::VecCompute};
    res->kernel_ = [res](Unit *c) {
        auto rdp = reinterpret_cast<double *>(c->acc_->comm_reg_.Get(res->rd_));
        auto rp0 = reinterpret_cast<double *>(c->acc_->comm_reg_.Get(res->rs0_));
        auto rp1 = reinterpret_cast<double *>(c->acc_->comm_reg_.Get(res->rs1_));
        uint32_t m = c->acc_->spec_reg_.Get(X_SIZE);
        uint32_t p = c->acc_->spec_reg_.Get(Y_SIZE);
        uint32_t n = c->acc_->spec_reg_.Get(Z_SIZE);
        for (int i = 0; i < m; ++i) {
            for (int j = 0; j < n; ++j) {
                rdp[i*n+j] = 0;
            }
        }
        for (int i = 0; i < m; ++i) {
            for (int l = 0; l < p; ++l) {
                for (int j = 0; j < n; ++j) {
                    rdp[i*n+j] += rp0[i*p+l]*rp1[l*n+j];
                }
            }
        }
        c->pc_ += 1;
    };
    res->rd_ = rd;
    res->rs0_ = rs0;
    res->rs1_ = rs1;
    res->name = "GEMM.F64";
    return res;
}
Instruction* Program::GemmC32(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs0, uint32_t rs1) {
    auto res = new AiInst{path, dri, dro, [](Unit*) {}, Tag::VecCompute};
    res->kernel_ = [res](Unit *c) {
        auto rdp = reinterpret_cast<float _Complex *>(c->acc_->comm_reg_.Get(res->rd_));
        auto rp0 = reinterpret_cast<float _Complex *>(c->acc_->comm_reg_.Get(res->rs0_));
        auto rp1 = reinterpret_cast<float _Complex *>(c->acc_->comm_reg_.Get(res->rs1_));
        uint32_t m = c->acc_->spec_reg_.Get(X_SIZE);
        uint32_t p = c->acc_->spec_reg_.Get(Y_SIZE);
        uint32_t n = c->acc_->spec_reg_.Get(Z_SIZE);
        for (int i = 0; i < m; ++i) {
            for (int j = 0; j < n; ++j) {
                rdp[i*n+j] = 0;
            }
        }
        for (int i = 0; i < m; ++i) {
            for (int l = 0; l < p; ++l) {
                for (int j = 0; j < n; ++j) {
                    rdp[i*n+j] += rp0[i*p+l]*rp1[l*n+j];
                }
            }
        }
        c->pc_ += 1;
    };
    res->rd_ = rd;
    res->rs0_ = rs0;
    res->rs1_ = rs1;
    res->name = "GEMM.C32";
    return res;
}
Instruction* Program::GemmC64(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs0, uint32_t rs1) {
    auto res = new AiInst{path, dri, dro, [](Unit*) {}, Tag::VecCompute};
    res->kernel_ = [res](Unit *c) {
        auto rdp = reinterpret_cast<double _Complex *>(c->acc_->comm_reg_.Get(res->rd_));
        auto rp0 = reinterpret_cast<double _Complex *>(c->acc_->comm_reg_.Get(res->rs0_));
        auto rp1 = reinterpret_cast<double _Complex *>(c->acc_->comm_reg_.Get(res->rs1_));
        uint32_t m = c->acc_->spec_reg_.Get(X_SIZE);
        uint32_t p = c->acc_->spec_reg_.Get(Y_SIZE);
        uint32_t n = c->acc_->spec_reg_.Get(Z_SIZE);
        for (int i = 0; i < m; ++i) {
            for (int j = 0; j < n; ++j) {
                rdp[i*n+j] = 0;
            }
        }
        for (int i = 0; i < m; ++i) {
            for (int l = 0; l < p; ++l) {
                for (int j = 0; j < n; ++j) {
                    rdp[i*n+j] += rp0[i*p+l]*rp1[l*n+j];
                }
            }
        }
        c->pc_ += 1;
    };
    res->rd_ = rd;
    res->rs0_ = rs0;
    res->rs1_ = rs1;
    res->name = "GEMM.C64";
    return res;
}

Instruction* Program::VmulC32(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs0, uint32_t rs1) {
    auto res = new AiInst{path, dri, dro, [](Unit*) {}, Tag::VecCompute};
    res->kernel_ = [res](Unit *c) {
        auto rdp = reinterpret_cast<float _Complex *>(c->acc_->comm_reg_.Get(res->rd_));
        auto rp0 = reinterpret_cast<float _Complex *>(c->acc_->comm_reg_.Get(res->rs0_));
        auto rp1 = reinterpret_cast<float _Complex *>(c->acc_->comm_reg_.Get(res->rs1_));
        uint32_t len = c->acc_->spec_reg_.Get(VLEN);
        /*if (len < 1000 || len > 64000) {
            std::cerr << "SIZE ERROS: for vmulc32(u, v), length of u,v should be [1k, 64K]" << std::endl;
            return;
        }*/
        for (uint32_t i = 0; i < len; ++i) {
            rdp[i] = rp0[i] * rp1[i];
        }
        c->pc_ += 1;
    };
    res->rd_ = rd;
    res->rs0_ = rs0;
    res->rs1_ = rs1;
    res->name = "VMULC32";
    return res;
}
Instruction* Program::VsubC32(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs0, uint32_t rs1) {
    auto res = new AiInst{path, dri, dro, [](Unit*) {}, Tag::VecCompute};
    res->kernel_ = [res](Unit *c) {
        auto rdp = reinterpret_cast<float _Complex*>(c->acc_->comm_reg_.Get(res->rd_));
        auto rp0 = reinterpret_cast<float _Complex*>(c->acc_->comm_reg_.Get(res->rs0_));
        auto rp1 = reinterpret_cast<float _Complex*>(c->acc_->comm_reg_.Get(res->rs1_));
        uint32_t len = c->acc_->spec_reg_.Get(VLEN);
        for (uint32_t i = 0; i < len; ++i) {
            rdp[i] = rp0[i] - rp1[i];
        }
        c->pc_ += 1;
    };
    res->rd_ = rd;
    res->rs0_ = rs0;
    res->rs1_ = rs1;
    res->name = "VSUBC32";
    return res;
}
Instruction* Program::VsubC64(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs0, uint32_t rs1) {
    auto res = new AiInst{path, dri, dro, [](Unit*) {}, Tag::VecCompute};
    res->kernel_ = [res](Unit *c) {
        auto rdp = reinterpret_cast<double _Complex*>(c->acc_->comm_reg_.Get(res->rd_));
        auto rp0 = reinterpret_cast<double _Complex*>(c->acc_->comm_reg_.Get(res->rs0_));
        auto rp1 = reinterpret_cast<double _Complex*>(c->acc_->comm_reg_.Get(res->rs1_));
        uint32_t len = c->acc_->spec_reg_.Get(VLEN);
        for (uint32_t i = 0; i < len; ++i) {
            rdp[i] = rp0[i] - rp1[i];
        }
        c->pc_ += 1;
    };
    res->rd_ = rd;
    res->rs0_ = rs0;
    res->rs1_ = rs1;
    res->name = "VSUBC64";
    return res;
}
Instruction* Program::VmuliC32(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs0, float _Complex imm) {
    auto res = new AiInst{path, dri, dro, [](Unit*) {}, Tag::VecCompute};
    res->kernel_ = [res, imm](Unit *c) {
        auto rdp = reinterpret_cast<float _Complex*>(c->acc_->comm_reg_.Get(res->rd_));
        auto rp0 = reinterpret_cast<float _Complex*>(c->acc_->comm_reg_.Get(res->rs0_));
        uint32_t len = c->acc_->spec_reg_.Get(VLEN);
        for (uint32_t i = 0; i < len; ++i) {
            rdp[i] = rp0[i] * imm;
        }
        c->pc_ += 1;
    };
    res->rd_ = rd;
    res->rs0_ = rs0;
    //res->rs1_ = rs1;
    res->name = "VMULIC32";
    return res;
}
Instruction* Program::VmuliC64(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs0, double _Complex imm) {
    auto res = new AiInst{path, dri, dro, [](Unit*) {}, Tag::VecCompute};
    res->kernel_ = [res, imm](Unit *c) {
        auto rdp = reinterpret_cast<double _Complex *>(c->acc_->comm_reg_.Get(res->rd_));
        auto rp0 = reinterpret_cast<double _Complex *>(c->acc_->comm_reg_.Get(res->rs0_));
        uint32_t len = c->acc_->spec_reg_.Get(VLEN);
        for (uint32_t i = 0; i < len; ++i) {
            rdp[i] = rp0[i] * imm;
        }
        c->pc_ += 1;
    };
    res->rd_ = rd;
    res->rs0_ = rs0;
    //res->rs1_ = rs1;
    res->name = "VMULIC64";
    return res;
}

Instruction* Program::cAddi(uint32_t rd, uint32_t rs0, int32_t imm) {
    return new BasicInst{[rd, rs0, imm](Unit* c) {
        uint32_t extent = c->acc_->spec_reg_.Get(EXTENT);
        uint32_t a_offset = c->acc_->spec_reg_.Get(ACCUM_OFFSET);
        uint32_t i_offset = c->acc_->spec_reg_.Get(INPUT_OFFSET);
        // uint32_t a_coeff = c->acc_->comm_reg_.Get(rd);
        // uint32_t i_coeff = c->acc_->comm_reg_.Get(rs0);
        auto ap = reinterpret_cast<ElemType*>(c->acc_->cache_.Get() + AccumBase) + a_offset * AccumBlock;
        auto ip = reinterpret_cast<ElemType*>(c->acc_->cache_.Get() + AccumBase) + i_offset * AccumBlock;
        for (uint32_t i = 0; i != extent * AccumBlock; ++i) {
            ap[i] = ip[i] + imm;
        }
        c->pc_ += 1;
    }};
}

Instruction* Program::cAdd(uint32_t rd, uint32_t rs0, uint32_t rs1) {
    return new BasicInst{[rd, rs1](Unit* c) {
        uint32_t extent = c->acc_->spec_reg_.Get(EXTENT);
        uint32_t a_offset = c->acc_->spec_reg_.Get(ACCUM_OFFSET);
        uint32_t i_offset = c->acc_->spec_reg_.Get(INPUT_OFFSET);
        // uint32_t a_coeff = c->acc_->comm_reg_.Get(rd);
        // uint32_t i_coeff = c->acc_->comm_reg_.Get(rs1);
        auto ap = reinterpret_cast<ElemType*>(c->acc_->cache_.Get() + AccumBase) + a_offset * AccumBlock;
        auto ip = reinterpret_cast<ElemType*>(c->acc_->cache_.Get() + AccumBase) + i_offset * AccumBlock;
        for (uint32_t i = 0; i != extent * AccumBlock; ++i) {
            ap[i] += ip[i];
        }
        c->pc_ += 1;
    }};
}

Instruction* Program::cMaxi(uint32_t rd, uint32_t rs0, int32_t imm) {
    return new BasicInst{[rd, rs0, imm](Unit* c) {
        uint32_t extent = c->acc_->spec_reg_.Get(EXTENT);
        uint32_t a_offset = c->acc_->spec_reg_.Get(ACCUM_OFFSET);
        uint32_t i_offset = c->acc_->spec_reg_.Get(INPUT_OFFSET);
        // uint32_t a_coeff = c->acc_->comm_reg_.Get(rd);
        // uint32_t i_coeff = c->acc_->comm_reg_.Get(rs0);
        auto ap = reinterpret_cast<ElemType*>(c->acc_->cache_.Get() + AccumBase) + a_offset * AccumBlock;
        auto ip = reinterpret_cast<ElemType*>(c->acc_->cache_.Get() + AccumBase) + i_offset * AccumBlock;
        ElemType rhs = static_cast<ElemType>(c->acc_->comm_reg_.Get(imm));
        for (uint32_t i = 0; i != extent * AccumBlock; ++i) {
            ap[i] = ((ip[i] < rhs) ? rhs : ip[i]);
        }
        c->pc_ += 1;
    }};
}

Instruction* Program::cMini(uint32_t rd, uint32_t rs0, int32_t imm) {
    return new BasicInst{[rd, rs0, imm](Unit* c) {
        uint32_t extent = c->acc_->spec_reg_.Get(EXTENT);
        uint32_t a_offset = c->acc_->spec_reg_.Get(ACCUM_OFFSET);
        uint32_t i_offset = c->acc_->spec_reg_.Get(INPUT_OFFSET);
        // uint32_t a_coeff = c->acc_->comm_reg_.Get(rd);
        // uint32_t i_coeff = c->acc_->comm_reg_.Get(rs0);
        auto ap = reinterpret_cast<ElemType*>(c->acc_->cache_.Get() + AccumBase) + a_offset * AccumBlock;
        auto ip = reinterpret_cast<ElemType*>(c->acc_->cache_.Get() + AccumBase) + i_offset * AccumBlock;
        ElemType rhs = static_cast<ElemType>(c->acc_->comm_reg_.Get(imm));
        for (uint32_t i = 0; i != extent * AccumBlock; ++i) {
            ap[i] = ((ip[i] > rhs) ? rhs : ip[i]);
        }
        c->pc_ += 1;
    }};
}

Instruction* Program::cShri(uint32_t rd, uint32_t rs0, int32_t imm) {
    return new BasicInst{[rd, rs0, imm](Unit* c) {
        uint32_t extent = c->acc_->spec_reg_.Get(EXTENT);
        uint32_t a_offset = c->acc_->spec_reg_.Get(ACCUM_OFFSET);
        uint32_t i_offset = c->acc_->spec_reg_.Get(INPUT_OFFSET);
        // uint32_t a_coeff = c->acc_->comm_reg_.Get(rd);
        // uint32_t i_coeff = c->acc_->comm_reg_.Get(rs0);
        auto ap = reinterpret_cast<ElemType*>(c->acc_->cache_.Get() + AccumBase) + a_offset * AccumBlock;
        auto ip = reinterpret_cast<ElemType*>(c->acc_->cache_.Get() + AccumBase) + i_offset * AccumBlock;
        ElemType rhs = static_cast<ElemType>(c->acc_->comm_reg_.Get(imm));
        for (uint32_t i = 0; i != extent * AccumBlock; ++i) {
            ap[i] = ip[i] >> rhs;
        }
        c->pc_ += 1;
    }};
}

Instruction* Program::cMax(uint32_t rd, uint32_t rs0, uint32_t rs1) {
    return new BasicInst{[](Unit* c) {
        c->pc_ += 1;
    }};
}

Instruction* Program::cMin(uint32_t rd, uint32_t rs0, uint32_t rs1) {
    return new BasicInst{[](Unit* c) {
        c->pc_ += 1;
    }};
}

Instruction* Program::cShr(uint32_t rd, uint32_t rs0, uint32_t rs1) {
    return new BasicInst{[](Unit* c) {
        c->pc_ += 1;
    }};
}

Instruction* Program::Vload(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs0,
                            uint32_t len) {
    return nullptr;
}

Instruction* Program::Vstore(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs0,
                             uint32_t len) {
    return nullptr;
}

Instruction* Program::MemSet(uint32_t dst, uint32_t len, uint32_t val) {
    return new BasicInst{[dst, len, val](Unit* c) {
        auto v = static_cast<ElemType>(c->acc_->comm_reg_.Get(val));
        auto block = c->acc_->comm_reg_.Get(len);
        auto p = reinterpret_cast<tai::ElemType*>(c->acc_->comm_reg_.Get(dst));
        for (uint32_t i = 0; i != block; ++i) p[i] = v;
        c->pc_ += 1;
    }};
}

Instruction* Program::Mload(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs0,
                            uint32_t len) {
    auto res = new AiInst{path, dri, dro, [](Unit*) {}, Tag::Load};
    res->kernel_ = [res](Unit* c) {
        auto block = c->acc_->comm_reg_.Get(res->rs1_);
        auto x_pad_before = c->acc_->spec_reg_.Get(tai::X_PAD_0);
        auto x_pad_after = c->acc_->spec_reg_.Get(tai::X_PAD_1);
        auto y_pad_before = c->acc_->spec_reg_.Get(tai::Y_PAD_0);
        auto y_pad_after = c->acc_->spec_reg_.Get(tai::Y_PAD_1);
        auto x_size = c->acc_->spec_reg_.Get(tai::X_SIZE);
        auto y_size = c->acc_->spec_reg_.Get(tai::Y_SIZE);
        auto x_stride = c->acc_->spec_reg_.Get(tai::X_STRIDE);

        ElemType* sram_addr = reinterpret_cast<ElemType*>(c->acc_->comm_reg_.Get(res->rd_));
        ElemType* dram_addr = reinterpret_cast<ElemType*>(c->acc_->comm_reg_.Get(res->rs0_));
        uint64_t x_total = block * (x_pad_before + x_size + x_pad_after);
        if (y_pad_before) {
            for (uint64_t i = 0; i != x_total; ++i) sram_addr[i] = 0;
            sram_addr += x_total;
        }
        for (uint32_t i = 0; i != y_size; ++i) {
            if (x_pad_before) {
                for (uint32_t i = 0; i != block; ++i) sram_addr[i] = 0;
                sram_addr += block;
            }
            memcpy(sram_addr, dram_addr, x_size * block * tai::ElemBytes);
            sram_addr += x_size * block;
            if (x_pad_after) {
                for (uint32_t i = 0; i != block; ++i) sram_addr[i] = 0;
                sram_addr += block;
            }
            dram_addr += x_stride * block;
        }
        if (y_pad_after) {
            for (uint64_t i = 0; i != x_total; ++i) sram_addr[i] = 0;
        }
    };
    res->rd_ = rd;
    res->rs0_ = rs0;
    res->rs1_ = len;
    res->name = "MLOAD";
    return res;
}

Instruction* Program::Gemm(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs0,
                           uint32_t rs1) {
    auto res = new AiInst{path, dri, dro, [](Unit*) {}, Tag::MatCompute};
    res->kernel_ = [res](Unit* c) {
        auto rst_acc = c->acc_->spec_reg_.Get(RESET_ACC);
        auto acc = reinterpret_cast<tai::ElemType*>(c->acc_->comm_reg_.Get(res->rd_));
        auto inp = reinterpret_cast<tai::ElemType*>(c->acc_->comm_reg_.Get(res->rs0_));
        auto wgt = reinterpret_cast<tai::ElemType*>(c->acc_->comm_reg_.Get(res->rs1_));

        for (size_t x = 0; x != tai::Batch; ++x) {
            for (size_t y = 0; y != tai::BlockOut; ++y) {
                if (rst_acc) acc[x * tai::BlockOut + y] = 0;
                for (size_t z = 0; z != tai::BlockIn; ++z) {
                    acc[x * tai::BlockOut + y] += inp[x * tai::BlockIn + z] * wgt[y * tai::BlockIn + z];
                }
            }
        }
        c->pc_ += 1;
    };
    res->rd_ = rd;
    res->rs0_ = rs0;
    res->rs1_ = rs1;
    res->name = "GEMM";
    return res;
}

Instruction* Program::Mstore(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs0,
                             uint32_t len) {
    auto res = new AiInst{path, dri, dro, [](Unit*) {}, Tag::Store};
    res->kernel_ = [res](Unit* c) {
        auto x_size = c->acc_->spec_reg_.Get(tai::X_SIZE);
        auto y_size = c->acc_->spec_reg_.Get(tai::Y_SIZE);
        auto x_stride = c->acc_->spec_reg_.Get(tai::X_STRIDE);
        auto dst =
                reinterpret_cast<ElemType*>(c->acc_->comm_reg_.Get(res->rd_));
        auto src =
                reinterpret_cast<ElemType*>(c->acc_->comm_reg_.Get(res->rs0_));
        auto block = c->acc_->comm_reg_.Get(res->rs1_);

        for (uint32_t i = 0; i != y_size; ++i) {
            for (uint32_t j = 0; j != x_size; ++j) {
                auto dram_base = i * x_stride + j;
                auto cache_base = i * x_size + j;
                for (uint32_t k = 0; k != block; ++k) {
                    dst[dram_base * block + k] = src[cache_base * block + k];
                }
            }
        }
    };
    res->rd_ = rd;
    res->rs0_ = rs0;
    res->rs1_ = len;
    res->name = "MSTORE";
    return res;
}

Instruction* Program::Tload(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs0,
                            uint32_t len) {
    return nullptr;
}

Instruction* Program::Tstore(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs0,
                             uint32_t len) {
    return nullptr;
}

Instruction* Program::Mma(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs0, uint32_t rs1) {
    auto res = new AiInst{path, dri, dro, [](Unit* c) {}, Tag::MatCompute};
    auto k = [res](Unit* c) {
        auto acc = reinterpret_cast<tai::ElemType*>(c->acc_->cache_.Get() + tai::AccumBase) +
                   c->acc_->comm_reg_.Get(res->rd_);
        auto wgt = reinterpret_cast<tai::ElemType*>(c->acc_->cache_.Get() + tai::ConstBase) +
                   c->acc_->comm_reg_.Get(res->rs1_);
        auto inp = reinterpret_cast<tai::ElemType*>(c->acc_->cache_.Get() + tai::InputBase) +
                   c->acc_->comm_reg_.Get(res->rs0_);
        auto m = c->acc_->spec_reg_.Get(MSIZE);
        auto n = c->acc_->spec_reg_.Get(NSIZE);

        for (size_t x = 0; x != m; ++x) {
            for (size_t y = 0; y != n; ++y) {
                acc[x * n + y] += inp[x * n + y] + wgt[y * n + y];
            }
        }
        c->pc_ += 1;
    };
    res->kernel_ = k;
    res->rd_ = rd;
    res->rs0_ = rs0;
    res->rs1_ = rs1;
    res->name = "MMA";
    return res;
}

Instruction* Program::Mmp(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs0, uint32_t rs1) {
    auto res = new AiInst{path, dri, dro, [](Unit* c) {}, Tag::MatCompute};
    auto k = [res](Unit* c) {
        auto acc = reinterpret_cast<tai::ElemType*>(c->acc_->cache_.Get() + tai::AccumBase) +
                   c->acc_->comm_reg_.Get(res->rd_);
        auto wgt = reinterpret_cast<tai::ElemType*>(c->acc_->cache_.Get() + tai::ConstBase) +
                   c->acc_->comm_reg_.Get(res->rs1_);
        auto inp = reinterpret_cast<tai::ElemType*>(c->acc_->cache_.Get() + tai::InputBase) +
                   c->acc_->comm_reg_.Get(res->rs0_);
        auto m = c->acc_->spec_reg_.Get(MSIZE);
        auto n = c->acc_->spec_reg_.Get(NSIZE);

        for (size_t x = 0; x != m; ++x) {
            for (size_t y = 0; y != n; ++y) {
                acc[x * n + y] += inp[x * n + y] * wgt[y * n + y];
            }
        }
        c->pc_ += 1;
    };
    res->kernel_ = k;
    res->rd_ = rd;
    res->rs0_ = rs0;
    res->rs1_ = rs1;
    res->name = "MMA";
    return res;
}

Instruction* Program::Smm(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs0, uint32_t rs1) {
    auto res = new AiInst{path, dri, dro, [](Unit* u) {}, Tag::MatCompute};
    auto k = [res](Unit* c) {
        auto acc = reinterpret_cast<tai::ElemType*>(c->acc_->cache_.Get() + tai::AccumBase) +
                   c->acc_->comm_reg_.Get(res->rd_);
        auto inp = reinterpret_cast<tai::ElemType*>(c->acc_->cache_.Get() + tai::InputBase) +
                   c->acc_->comm_reg_.Get(res->rs0_);
        auto m = c->acc_->spec_reg_.Get(MSIZE);
        auto n = c->acc_->spec_reg_.Get(NSIZE);

        for (size_t x = 0; x != m; ++x) {
            for (size_t y = 0; y != n; ++y) {
                acc[x * n + y] += inp[x * n + y] * res->rs1_;
            }
        }
    };
    res->kernel_ = k;
    res->rd_ = rd;
    res->rs0_ = rs0;
    res->rs1_ = rs1;
    res->name = "SMM";
    return res;
}

Instruction* Program::Mclip(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs0,
                            uint32_t rs1) {
    auto res = new AiInst{path, dri, dro, [](Unit*) {}, Tag::MatCompute};
    auto k = [res](Unit* c) {
        auto acc = reinterpret_cast<tai::ElemType*>(c->acc_->cache_.Get() + tai::AccumBase) +
                   c->acc_->comm_reg_.Get(res->rd_);
        float upper = (res->rs1_) >> 16;
        float lower = (res->rs1_) & 0x0000FFFF;
        auto inp = reinterpret_cast<tai::ElemType*>(c->acc_->cache_.Get() + tai::InputBase) +
                   c->acc_->comm_reg_.Get(res->rs0_);
        auto m = c->acc_->spec_reg_.Get(MSIZE);
        auto n = c->acc_->spec_reg_.Get(NSIZE);

        for (uint32_t i = 0; i != m; ++i) {
            for (uint32_t j = 0; j != n; ++j) {
                if (inp[i * n + j] > upper) acc[i * n + j] = upper;
                if (inp[i * n + j] < lower) acc[i * n + j] = lower;
            }
        }
        c->pc_ += 1;
    };
    res->kernel_ = k;
    res->rd_ = rd;
    res->rs0_ = rs0;
    res->rs1_ = rs1;
    res->name = "MCLIP";
    return res;
}

Instruction* Program::Halt() { return nullptr; }

Instruction* Program::Subi(uint32_t rd, uint32_t rs0, int imm) {
    return new BasicInst{[rd, rs0, imm](Unit* c) {
        c->acc_->comm_reg_.Set(rd, c->acc_->comm_reg_.Get(rs0) - imm);
        c->pc_ += 1;
    }};
}

Instruction* Program::Sub(uint32_t rd, uint32_t rs0, uint32_t rs1) {
    return new BasicInst{[rd, rs0, rs1](Unit* c) {
        c->acc_->comm_reg_.Set(rd, c->acc_->comm_reg_.Get(rs0) - c->acc_->comm_reg_.Get(rs1));
        c->pc_ += 1;
    }};
}

Instruction* Program::Mvp(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs0, uint32_t rs1) {
    return nullptr;
}

Instruction* Program::Display(const std::string& msg, uint32_t rs0) {
    return new BasicInst{[msg, rs0](Unit* c) {
        std::cerr << msg << ", Reg " << rs0 << ": " << c->acc_->comm_reg_.Get(rs0) << std::endl;
        c->pc_ += 1;
    }};
}


Instruction* Program::Conv(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs0, uint32_t rs1) {
    auto res = new AiInst{path, dri, dro, [](Unit *) {}, Tag::VecCompute};
    res->kernel_ = [res](Unit *c) {
        auto rdp = reinterpret_cast<float*>(c->acc_->comm_reg_.Get(res->rd_));
        auto rp0 = reinterpret_cast<float*>(c->acc_->comm_reg_.Get(res->rs0_));
        auto rp1 = reinterpret_cast<float*>(c->acc_->comm_reg_.Get(res->rs1_));

        uint32_t ulen = c->acc_->spec_reg_.Get(ULEN);
        uint32_t vlen = c->acc_->spec_reg_.Get(VLEN);
        uint32_t len = ulen + vlen  - 1;/*
        if (ulen < 1000 || ulen > 64000) {
            std::cerr << "SIZE ERROS: for conv(u, v), length of u should be [1k, 64K]" << std::endl;
            return;
        }
        if (vlen < 16 || vlen > 128) {
            std::cerr << "SIZE ERROS: for conv(u, v), length of v should be [16, 128]" << std::endl;
            return;
        }*/

        for (uint32_t i = 0; i < len; ++i) 
            rdp[i] = 0;
        for (uint32_t i = 0; i < ulen; ++i) {
            for (uint32_t j = 0; j < vlen; ++j) {
                rdp[i+j] += rp0[i] * rp1[j];
            }
        }
        c->pc_ += 1;
    };
    res->rd_ = rd;
    res->rs0_ = rs0;
    res->rs1_ = rs1;
    res->name = "CONV";
    return res;
}





Instruction* Program::Fft(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs) {
    auto res = new AiInst{path, dri, dro, [](Unit *) {}, Tag::VecCompute};
    res->kernel_ = [res](Unit *c) {
        auto rdp = reinterpret_cast<float _Complex*>(c->acc_->comm_reg_.Get(res->rd_));
        //auto rp0 = reinterpret_cast<float *>(c->acc_->comm_reg_.Get(res->rs0_));
        auto rp0 = reinterpret_cast<float _Complex*>(c->acc_->comm_reg_.Get(res->rs0_));

        uint32_t len = c->acc_->spec_reg_.Get(VLEN);
        /*
        if (len < 1000 || len > 64000) {
            std::cerr << "SIZE ERROS: for fft(x), length of x should be [1k, 64K]" << std::endl;
            return;
        }*/

        for (uint32_t i = 0; i < len; ++i) {
            rdp[i] = 0;
            for (uint32_t j = 0; j < len; ++j) {
                rdp[i] += rp0[j] * cexpf(-2*PI*I*i*j/len);
            }
        }
        c->pc_ += 1;
    };
    res->rd_ = rd;
    res->rs0_ = rs;
    res->name = "FFT";
    return res;
}

Instruction* Program::Ifft(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs) {
    auto res = new AiInst{ path, dri, dro, [](Unit*) {}, Tag::VecCompute };
    res->kernel_ = [res](Unit* c) {
        auto rdp = reinterpret_cast<float _Complex*>(c->acc_->comm_reg_.Get(res->rd_));
        auto rp0 = reinterpret_cast<float _Complex*>(c->acc_->comm_reg_.Get(res->rs0_));

        uint32_t len = c->acc_->spec_reg_.Get(VLEN);
        /*
        if (len < 1000 || len > 64000) {
            std::cerr << "SIZE ERROS: for Ifft(x), length of x should be [1k, 64K]" << std::endl;
            return;
        }*/

        for (uint32_t i = 0; i < len; ++i) {
            rdp[i] = 0;
            for (uint32_t j = 0; j < len; ++j) {
                //rdp[i] += rp0[j] * (1 / len) * cexpf(2 * PI * I * i * j / len);
                rdp[i] += (rp0[j] * cexpf(2 * PI * I * i * j / len));
            }
            rdp[i] /= len;
        }
        c->pc_ += 1;
    };
    res->rd_ = rd;
    res->rs0_ = rs;
    res->name = "IFFT";
    return res;
}


Instruction* Program::Ddc(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs) {
    auto res = new AiInst{ path, dri, dro, [](Unit*) {}, Tag::VecCompute };
    res->kernel_ = [res](Unit* c) {
        auto rdp = reinterpret_cast<float _Complex*>(c->acc_->comm_reg_.Get(res->rd_));
        auto rp0 = reinterpret_cast<float*>(c->acc_->comm_reg_.Get(res->rs0_));


        uint32_t fc = c->acc_->spec_reg_.Get(ULEN);
        uint32_t Ts = c->acc_->spec_reg_.Get(VLEN);
        uint32_t x_size = c->acc_->spec_reg_.Get(X_SIZE);
        

        for (uint32_t i = 0; i < x_size; ++i)
 {            
                rdp[i] = rp0[i] * cexpf(-2 * PI * I * i * fc * Ts);
            
        }
        c->pc_ += 1;
    };
    res->rd_ = rd;
    res->rs0_ = rs;
    
    res->name = "DDC";
    return res;
}



Instruction* Program::Fir(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs0, uint32_t rs1) {
    auto res = new AiInst{ path, dri, dro, [](Unit*) {}, Tag::VecCompute };
    res->kernel_ = [res](Unit* c) {
        auto rdp = reinterpret_cast<int32_t*>(c->acc_->comm_reg_.Get(res->rd_));
        auto rp0 = reinterpret_cast<int32_t*>(c->acc_->comm_reg_.Get(res->rs0_));
        auto rp1 = reinterpret_cast<int32_t*>(c->acc_->comm_reg_.Get(res->rs1_));

        uint32_t ulen = c->acc_->spec_reg_.Get(ULEN);
        uint32_t vlen = c->acc_->spec_reg_.Get(VLEN);
        uint32_t len = ulen + vlen - 1;

        for (uint32_t i = 0; i < len; ++i)
            rdp[i] = 0;
        for (uint32_t i = 0; i < ulen; ++i) {
            for (uint32_t j = 0; j < vlen; ++j) {
                rdp[i + j] += rp0[i] * rp1[j];
            }
        }
        c->pc_ += 1;
    };
    res->rd_ = rd;
    res->rs0_ = rs0;
    res->rs1_ = rs1;
    res->name = "FIR";
    return res;
}


Instruction* Program::Extr(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs) {
    auto res = new AiInst{ path, dri, dro, [](Unit*) {}, Tag::VecCompute };
    res->kernel_ = [res](Unit* c) {
        auto rdp = reinterpret_cast<int32_t*>(c->acc_->comm_reg_.Get(res->rd_));
        auto rp0 = reinterpret_cast<int32_t*>(c->acc_->comm_reg_.Get(res->rs0_));


        uint32_t ulen = c->acc_->spec_reg_.Get(ULEN);
        uint32_t x_size = c->acc_->spec_reg_.Get(X_SIZE);
        uint32_t vlen = (ulen-1)/(x_size+1) + 1;

        for (uint32_t i = 0, j = 0; i < ulen, j < vlen; i = i + x_size+1,j++)
        {
            rdp[j] = rp0[i];

        }
        c->pc_ += 1;
    };
    res->rd_ = rd;
    res->rs0_ = rs;

    res->name = "EXTR";
    return res;
}




Label::Label(const std::string& t) {
    type_ = Type::Label;
    text_ = t;
}

BasicInst::BasicInst(std::function<void(Unit*)> k) {
    type_ = Type::BasicInst;
    kernel_ = std::move(k);
}

AiInst::AiInst(std::function<void(Unit*)> k, Tag t) {
    type_ = Type::AiInst;
    tag_ = t;
    driver_ = Drive::Inst;
    driven_ = Drive::Mem;
    kernel_ = std::move(k);
}

AiInst::AiInst(int p, Drive dri, Drive dro, std::function<void(Unit*)> k, Tag t) {
    type_ = Type::AiInst;
    tag_ = t;
    path_ = p;
    driver_ = dri;
    driven_ = dro;
    kernel_ = std::move(k);
}
