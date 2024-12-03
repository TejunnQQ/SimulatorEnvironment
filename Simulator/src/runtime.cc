#include <algorithm>
#include <iostream>
#include <stdlib.h>
#include <complex.h>
#include <string.h>
#include <map>
#include "../include/tai_sim.h"
#include "../include/runtime.h"

#define _API_CALL
typedef void* HANDLE;

namespace tai {

    struct CommandQueue {
        CommandQueue():
                acc(std::make_shared<tai::Accelerator>()),
                prog((std::make_shared<tai::Program>())) {
        }

        ~CommandQueue() {}

        void Init() {
            prog->CreateFunc("do_load_data", {
                    prog->Dmovo(tai::X_PAD_0, 0),
                    prog->Dmovo(tai::X_PAD_1, 1),
                    prog->Dmovo(tai::Y_PAD_0, 2),
                    prog->Dmovo(tai::Y_PAD_1, 3),
                    prog->Dmovo(tai::X_SIZE, 4),
                    prog->Dmovo(tai::Y_SIZE, 5),
                    prog->Dmovo(tai::X_STRIDE, 6),
                    prog->Mload(0, tai::Drive::Inst, tai::Drive::Mem, 7, 8, 9),
                    prog->Fence(0),
                    prog->Ret(),
            });

            prog->CreateFunc("do_memset", {
                    prog->MemSet(0, 1, 2),
                    prog->Ret(),
            });

            prog->CreateFunc("do_gemm", {
                    prog->Gemm(0, tai::Drive::Inst, tai::Drive::Mem, 0, 1, 2),
                    prog->Ret(),
            });

            prog->CreateFunc("do_mini", {
                    prog->Dmovo(ACCUM_OFFSET, 0),
                    prog->Dmovo(INPUT_OFFSET, 1),
                    prog->Dmovo(CONST_OFFSET, 2),
                    prog->Dmovo(EXTENT, 3),
                    prog->cMini(4, 5, 6),
                    prog->Ret(),
            });

            prog->CreateFunc("do_maxi", {
                    prog->Dmovo(ACCUM_OFFSET, 0),
                    prog->Dmovo(INPUT_OFFSET, 1),
                    prog->Dmovo(CONST_OFFSET, 2),
                    prog->Dmovo(EXTENT, 3),
                    prog->cMaxi(4, 5, 6),
                    prog->Ret(),
            });

            prog->CreateFunc("do_add", {
                    prog->Dmovo(ACCUM_OFFSET, 0),
                    prog->Dmovo(INPUT_OFFSET, 1),
                    prog->Dmovo(CONST_OFFSET, 2),
                    prog->Dmovo(EXTENT, 3),
                    prog->cAdd(4, 5, 6),
                    prog->Ret(),
            });

            prog->CreateFunc("do_shri", {
                    prog->Dmovo(ACCUM_OFFSET, 0),
                    prog->Dmovo(INPUT_OFFSET, 1),
                    prog->Dmovo(CONST_OFFSET, 2),
                    prog->Dmovo(EXTENT, 3),
                    prog->cShri(4, 5, 6),
                    prog->Ret(),
            });

            prog->CreateFunc("do_store_data", {
                    prog->Fence(0),
                    prog->Dmovo(tai::X_SIZE, 0),
                    prog->Dmovo(tai::Y_SIZE, 1),
                    prog->Dmovo(tai::X_STRIDE, 2),
                    prog->Mstore(0, tai::Drive::Inst, tai::Drive::Mem, 3, 4, 5),
                    prog->Fence(0),
                    prog->Ret(),
            });
        }

        void LoadBufffer(void* src_dram_addr, uint32_t src_elem_offset, uint32_t x_size, uint32_t y_size,
                         uint32_t x_stride, uint32_t x_pad_before, uint32_t y_pad_before,
                         uint32_t x_pad_after, uint32_t y_pad_after, uint32_t dst_sram_index,
                         uint32_t dst_memory_type) {
            uint8_t* dst = acc->cache_.Get();
            uint8_t* src = static_cast<uint8_t*>(src_dram_addr);
            uint32_t block = 0;
            switch (dst_memory_type) {
                case tai::MemIdInput: {
                    block = tai::InputBlock;
                    dst += tai::InputBase;
                    break;
                }
                case tai::MemIdConst: {
                    block = tai::ConstBlock;
                    dst += tai::ConstBase;
                    break;
                }
                case tai::MemIdAccum: {
                    block = tai::AccumBlock;
                    dst += tai::AccumBase;
                    break;
                }
                default:
                    // "Memory id not recognized: " << dst_memory_type;
                    break;
            }
            ElemType* dram_addr = reinterpret_cast<ElemType*>(src) + src_elem_offset * block;
            ElemType* sram_addr = reinterpret_cast<ElemType*>(dst) + dst_sram_index * block;
            insq.push_back(prog->Movi(128, x_pad_before));
            insq.push_back(prog->Movi(129, x_pad_after));
            insq.push_back(prog->Movi(130, y_pad_before));
            insq.push_back(prog->Movi(131, y_pad_after));
            insq.push_back(prog->Movi(132, x_size));
            insq.push_back(prog->Movi(133, y_size));
            insq.push_back(prog->Movi(134, x_stride));
            insq.push_back(prog->Movi(135, reinterpret_cast<int64_t>(sram_addr)));
            insq.push_back(prog->Movi(136, reinterpret_cast<int64_t>(dram_addr)));
            insq.push_back(prog->Movi(137, block));
            insq.push_back(prog->Call("do_load_data", "MPU", 0, 128, 10));
        }

        void StoreBuffer(uint64_t src_elem_offset, uint32_t src_memory_type, void* dst_dram_addr,
                         uint32_t dst_elem_offset, uint32_t x_size, uint32_t y_size, uint32_t x_stride) {
            auto src = reinterpret_cast<ElemType*>(acc->cache_.Get() + AccumBase) + src_elem_offset;
            auto dst = reinterpret_cast<ElemType*>(dst_dram_addr) + dst_elem_offset * AccumBlock;

            insq.push_back(prog->Movi(138, x_size));
            insq.push_back(prog->Movi(139, y_size));
            insq.push_back(prog->Movi(140, x_stride));
            insq.push_back(prog->Movi(141, reinterpret_cast<int64_t>(dst)));
            insq.push_back(prog->Movi(142, reinterpret_cast<int64_t>(src)));
            insq.push_back(prog->Movi(143, AccumBlock));
            insq.push_back(prog->Call("do_store_data", "MPU", 0, 138, 6));
        }

        void MemReset(uint32_t reset_out, uint64_t dst_index, uint32_t dst_offset, uint64_t src_index,
                      uint32_t src_offset, uint64_t wgt_index, uint32_t wgt_offset) {
            if (reset_out) {
                auto p = reinterpret_cast<tai::ElemType*>(acc->cache_.Get() + tai::AccumBase) + dst_offset;
                insq.push_back(prog->Movi(64, reinterpret_cast<int64_t>(p)));
                insq.push_back(prog->Movi(65, AccumBlock));
                insq.push_back(prog->Movi(66, 0));
                insq.push_back(prog->Call("do_memset", "MPU", 0, 64, 3));
            }
        }

        void GemmOp(uint32_t rst_acc, uint64_t dst_index, uint32_t dst_offset, uint64_t src_index,
                    uint32_t src_offset, uint64_t wgt_index, uint32_t wgt_offset) {
            auto accum = reinterpret_cast<tai::ElemType*>(acc->cache_.Get() + tai::AccumBase) + dst_offset;
            auto inp = reinterpret_cast<tai::ElemType*>(acc->cache_.Get() + tai::InputBase) + src_offset;
            auto wgt = reinterpret_cast<tai::ElemType*>(acc->cache_.Get() + tai::ConstBase) + wgt_offset;

            insq.push_back(prog->Movid(tai::RESET_ACC, rst_acc));
            insq.push_back(prog->Movi(160, reinterpret_cast<int64_t>(accum)));
            insq.push_back(prog->Movi(161, reinterpret_cast<int64_t>(inp)));
            insq.push_back(prog->Movi(162, reinterpret_cast<int64_t>(wgt)));
            insq.push_back(prog->Call("do_gemm", "MPU", 0, 160, 3));
        }

        void PushCuInsts(uint32_t opcode, uint32_t extent, uint32_t reset, uint32_t dst_coeff,
                         uint32_t dst_offset, uint32_t src_coeff, uint32_t src_offset, uint32_t wgt_coeff,
                         uint32_t wgt_offset, uint32_t use_imm, int32_t imm) {
            insq.push_back(prog->Movi(198, dst_offset));
            insq.push_back(prog->Movi(199, src_offset));
            insq.push_back(prog->Movi(200, wgt_offset));
            insq.push_back(prog->Movi(201, extent));
            insq.push_back(prog->Movi(202, dst_coeff));
            insq.push_back(prog->Movi(203, src_coeff));
            insq.push_back(prog->Movi(204, imm));

            switch (opcode) {
                case uint32_t(Opcode::min): {
                    if(use_imm) insq.push_back(prog->Call("do_mini", "MPU", 0, 198, 7));
                    else        /* todo */;
                    break;
                }
                case uint32_t(Opcode::max): {
                    if(use_imm) insq.push_back(prog->Call("do_maxi", "MPU", 0, 198, 7));
                    else        /* todo */;
                    break;
                }
                case uint32_t(Opcode::add): {
                    if(use_imm) /* todo */;
                    else        insq.push_back(prog->Call("do_add", "MPU", 0, 198, 7));
                }
                case uint32_t(Opcode::shr): {
                    if(use_imm) insq.push_back(prog->Call("do_shri", "MPU", 0, 198, 7));
                    else        /* todo */;
                    break;
                }
                default:
                    // "Unknown opcode";
                    break;
            }
        }
        



        void* AllocBuffer(size_t size) {
            return acc->dram_.Alloc(size);
        }


        void FreeBuffer(void* buffer) {
            acc->dram_.Free(buffer);
        }

        void MemCopyFromHost(void* dst, const void* src, size_t size) {
            memcpy(dst, src, size);
        }

        void MemCopyToHost(void* dst, const void* src, size_t size) {
            memcpy(dst, src, size);
        }







        

        

        void Synchronize() {
            insq.push_back(prog->Ret());
            prog->CreateFunc("MAIN", std::move(insq));
            prog->Build();
            acc->Run(prog);
            prog->Clear();
        }

        void PushInst(Instruction *inst) {
            insq.push_back(inst);
        }

        static std::shared_ptr<CommandQueue>& ThreadLocal() {
            static std::shared_ptr<CommandQueue> inst = std::make_shared<CommandQueue>();
            if (inst == nullptr) {
                inst = std::make_shared<CommandQueue>();
            }
            return inst;
        }

        std::shared_ptr<tai::Program> GetProgram() {
            return prog;
        }

    private:
        std::shared_ptr<tai::Accelerator> acc;
        std::shared_ptr<tai::Program> prog;
        std::vector<tai::Instruction*> insq;
    };

}  // namespace tai




void* fpga_alloc_dma(unsigned long long len32, void* buffer = NULL, HANDLE share_dma = NULL) {
    return tai::CommandQueue::ThreadLocal()->AllocBuffer(len32);
}


void fpga_send(const void* from, size_t from_offset, HANDLE dma, unsigned long long offset, unsigned long long len32,
    unsigned int last = 1, unsigned int mm_addr = 0, unsigned int mm_addr_inc = 0, unsigned int timeout = 0) {
    //memcpy(dst, src, size);
    //TAIBufferCopy(addr8, 0, dma, 0, len32, 1);
    tai::CommandQueue::ThreadLocal()->MemCopyFromHost(
        static_cast<char*>(dma) + offset, static_cast<const char*>(from) + from_offset, len32);

}

void _API_CALL fpga_recv(const void* from, size_t from_offset, HANDLE dma, unsigned long long offset, unsigned long long len32,
    unsigned int last = 1, unsigned int mm_addr = 0, unsigned int mm_addr_inc = 0, unsigned int timeout = 0) {
    //memcpy(dst, src, size);
    tai::CommandQueue::ThreadLocal()->MemCopyToHost(
        static_cast<char*>(dma) + offset, static_cast<const char*>(from) + from_offset, len32);
}


void fpga_free_dma(HANDLE dma) {
    tai::CommandQueue::ThreadLocal()->FreeBuffer(dma);
}




typedef struct FUNC {
    std::string func_name;
    unsigned int param_0;
    unsigned int param_1;
    unsigned int param_2;
    unsigned int param_3;
    unsigned int param_4;
    unsigned int param_5;
    unsigned int param_6;
    //vector<string> parameters = {};

}FUNC;


typedef std::map<unsigned int, FUNC*> irq_HASH_SET;
//typedef std::map<char, FUNC*> irq_HASH_SET;

static irq_HASH_SET irqSet;
/*
template <class T>
T print(T arg)
{
    T aaa = arg;

    return aaa;
}*/

//template <class ...Args>
//unsigned int _API_CALL fpga_set_irq_callback(unsigned int user_irq_num,char* func, Args...args) {
//unsigned int _API_CALL fpga_set_irq_callback(unsigned int user_irq_num,char* func, size_t argsize, unsigned int* a = 0) {
unsigned int _API_CALL fpga_set_irq_callback(unsigned int user_irq_num,char* func) {

    while (user_irq_num&&func) {
        //std::vector<int> a = { (print(args))... };
        //std::vector<int> b = { (print(args),0)... };
        //unsigned int len_a = sizeof(a) / 4;

        FUNC*& ref = irqSet[user_irq_num];
        char params[20];
        if (ref == 0)
        {
            ref = new FUNC();
            ref->func_name = func;/*
            for (int ind = 0; ind != len_a; ++ind) {
                switch (ind) {
                case 0: ref->param_0 = a[0];
                case 1: ref->param_1 = a[1];
                case 2: ref->param_2 = a[2];
                case 3: ref->param_3 = a[3];
                case 4: ref->param_4 = a[4];
                case 5: ref->param_5 = a[5];
                case 6: ref->param_6 = a[6];
                }

            }*/

        }

        return 1;
    }
}


void fpga_open() {
    fpga_set_irq_callback(1, "Fft");
    fpga_set_irq_callback(2, "Ifft");
    fpga_set_irq_callback(3, "Extr");
    fpga_set_irq_callback(4, "VmulC32");
    fpga_set_irq_callback(5, "Fir");
    fpga_set_irq_callback(6, "Ddc");
}



//template <class ...Args>
unsigned int _API_CALL fpga_wait_irq(unsigned int user_irq_num, unsigned int timeout, void** args) {
    while (user_irq_num && timeout) {
        /*
        int a[] = { (print(args))... };
        int b[] = { (print(args),0)... };
        unsigned int len_a = sizeof(b) / 4;
        vector<int> c{ (print(args))... };

        int bq[5] = { 12,22,32,42,52 };*/

        auto iter = irqSet.find(user_irq_num);

        std::string s1 = iter->second->func_name;
        char inst1[50] = { 0 };
        

        //if (strcmp(s1, "Fft") == 0) {
        if (s1 == "Fft") {
            void* a_addr = args[0];
            void* res_addr = args[1];
            auto argsSize = reinterpret_cast<size_t*>(args[2]);
            //unsigned int s2 = iter->second->param_0;
            
            //TAIPushMoviOp(160, (uint64_t)res_addr);
            snprintf(inst1, sizeof(inst1), "MOVI $0x%x, #0x%lx", (uint32_t)160, (int64_t)res_addr);
            TAIPushInst(inst1);


            //TAIPushMoviOp(161, (uint64_t)a_addr); 
            snprintf(inst1, sizeof(inst1), "MOVI $0x%x, #0x%lx", (uint32_t)161, (int64_t)a_addr);
            TAIPushInst(inst1);


            snprintf(inst1, sizeof(inst1), "MOVID $%s, #0x%lx", "VLEN", (int64_t)argsSize);
            TAIPushInst(inst1);

            snprintf(inst1, sizeof(inst1), "FFT #0x0, #INST, #MEM, $0x%x, $0x%x", (uint32_t)160, (uint32_t)161);
            TAIPushInst(inst1);


            TAISynchronize();


        }

        if (s1 == "Ifft") {
            void* a_addr = args[0];
            void* res_addr = args[1];
            auto argsSize = reinterpret_cast<size_t*>(args[2]);
            //unsigned int s2 = iter->second->param_0;

            snprintf(inst1, sizeof(inst1), "MOVI $0x%x, #0x%lx", (uint32_t)160, (int64_t)res_addr);
            TAIPushInst(inst1);

            snprintf(inst1, sizeof(inst1), "MOVI $0x%x, #0x%lx", (uint32_t)161, (int64_t)a_addr);
            TAIPushInst(inst1);

            snprintf(inst1, sizeof(inst1), "MOVID $%s, #0x%lx", "VLEN", (int64_t)argsSize);
            TAIPushInst(inst1);

            snprintf(inst1, sizeof(inst1), "IFFT #0x0, #INST, #MEM, $0x%x, $0x%x", (uint32_t)160, (uint32_t)161);
            TAIPushInst(inst1);

            TAISynchronize();

        }
        
        if (s1 == "Extr") {
            void* a_addr = args[0];
            void* res_addr = args[1];
            auto s2 = reinterpret_cast<size_t*>(args[2]);
            auto s3 = reinterpret_cast<size_t*>(args[3]);

            //unsigned int s2 = iter->second->param_0;
            //unsigned int s3 = iter->second->param_1;


            snprintf(inst1, sizeof(inst1), "MOVI $0x%x, #0x%lx", (uint32_t)160, (int64_t)res_addr);
            TAIPushInst(inst1);

            snprintf(inst1, sizeof(inst1), "MOVI $0x%x, #0x%lx", (uint32_t)161, (int64_t)a_addr);
            TAIPushInst(inst1);


            snprintf(inst1, sizeof(inst1), "MOVID $%s, #0x%lx", "ULEN", (int64_t)s2);
            TAIPushInst(inst1);

            snprintf(inst1, sizeof(inst1), "MOVID $%s, #0x%lx", "X_SIZE", (int64_t)s3);
            TAIPushInst(inst1);

            snprintf(inst1, sizeof(inst1), "EXTR #0x0, #INST, #MEM, $0x%x, $0x%x", (uint32_t)160, (uint32_t)161);
            TAIPushInst(inst1);

            TAISynchronize();

        }
       
        if (s1 == "VmulC32") {
            void* a_addr = args[0];
            void* c_addr = args[1];
            void* res_addr = args[2];
            auto argsSize = reinterpret_cast<size_t*>(args[3]);

            //unsigned int argsSize = iter->second->param_0;

            snprintf(inst1, sizeof(inst1), "MOVI $0x%x, #0x%lx", (uint32_t)160, (int64_t)res_addr);
            TAIPushInst(inst1);

            snprintf(inst1, sizeof(inst1), "MOVI $0x%x, #0x%lx", (uint32_t)161, (int64_t)a_addr);
            TAIPushInst(inst1);

            snprintf(inst1, sizeof(inst1), "MOVI $0x%x, #0x%lx", (uint32_t)162, (int64_t)c_addr);
            TAIPushInst(inst1);

            snprintf(inst1, sizeof(inst1), "MOVID $%s, #0x%lx", "VLEN", (int64_t)argsSize);
            TAIPushInst(inst1);

            snprintf(inst1, sizeof(inst1), "VMUL.C32 #0x0, #INST, #MEM, $0x%x, $0x%x, $0x%x", (uint32_t)160, (uint32_t)161, (uint32_t)162);
            TAIPushInst(inst1);


            TAISynchronize();

        }
        
        if (s1 == "Fir") {

            void* a_addr = args[0];
            void* c_addr = args[1];
            void* res_addr = args[2];
            auto s2 = reinterpret_cast<size_t*>(args[3]);
            auto s3 = reinterpret_cast<size_t*>(args[4]);

            //unsigned int s2 = iter->second->param_0;
            //unsigned int s3 = iter->second->param_1;


            snprintf(inst1, sizeof(inst1), "MOVI $0x%x, #0x%lx", (uint32_t)160, (int64_t)res_addr);
            TAIPushInst(inst1);

            snprintf(inst1, sizeof(inst1), "MOVI $0x%x, #0x%lx", (uint32_t)161, (int64_t)a_addr);
            TAIPushInst(inst1);

            snprintf(inst1, sizeof(inst1), "MOVI $0x%x, #0x%lx", (uint32_t)162, (int64_t)c_addr);
            TAIPushInst(inst1);

            snprintf(inst1, sizeof(inst1), "MOVID $%s, #0x%lx", "ULEN", (int64_t)s2);
            TAIPushInst(inst1);

            snprintf(inst1, sizeof(inst1), "MOVID $%s, #0x%lx", "VLEN", (int64_t)s3);
            TAIPushInst(inst1);

            snprintf(inst1, sizeof(inst1), "CONV #0x0, #INST, #MEM, $0x%x, $0x%x, $0x%x", (uint32_t)160, (uint32_t)161, (uint32_t)162);
            TAIPushInst(inst1);


            TAISynchronize();
        }
        
        if (s1 == "Ddc") {
            void* a_addr = args[0];
            void* res_addr = args[1];

            auto argsSize = reinterpret_cast<size_t*>(args[2]);
            auto fc = reinterpret_cast<size_t*>(args[3]);
            auto Ts = reinterpret_cast<size_t*>(args[4]);

            //unsigned int argsSize = iter->second->param_0;
            //unsigned int fc = iter->second->param_1;
            //unsigned int Ts = iter->second->param_2;


            snprintf(inst1, sizeof(inst1), "MOVI $0x%x, #0x%lx", (uint32_t)160, (int64_t)res_addr);
            TAIPushInst(inst1);

            snprintf(inst1, sizeof(inst1), "MOVI $0x%x, #0x%lx", (uint32_t)161, (int64_t)a_addr);
            TAIPushInst(inst1);

            snprintf(inst1, sizeof(inst1), "MOVID $%s, #0x%lx", "ULEN", (int64_t)fc);
            TAIPushInst(inst1);

            snprintf(inst1, sizeof(inst1), "MOVID $%s, #0x%lx", "VLEN", (int64_t)Ts);
            TAIPushInst(inst1);

            snprintf(inst1, sizeof(inst1), "MOVID $%s, #0x%lx", "X_SIZE", (int64_t)argsSize);
            TAIPushInst(inst1);

            snprintf(inst1, sizeof(inst1), "DDC #0x0, #INST, #MEM, $0x%x, $0x%x", (uint32_t)160, (uint32_t)161);
            TAIPushInst(inst1);

            TAISynchronize();
        }/**/

        return 0;
    }
}







/*
void* TAIBufferAlloc(size_t size) { return tai::CommandQueue::ThreadLocal()->AllocBuffer(size); }


void TAIBufferCopy(const void* from, size_t from_offset, void* to, size_t to_offset, size_t size,
                   int kind_mask) {
    if (kind_mask & 2) {
        tai::CommandQueue::ThreadLocal()->MemCopyToHost(
                static_cast<char*>(to) + to_offset, static_cast<const char*>(from) + from_offset, size);
    }
    if (kind_mask & 1) {
        tai::CommandQueue::ThreadLocal()->MemCopyFromHost(
                static_cast<char*>(to) + to_offset, static_cast<const char*>(from) + from_offset, size);
    }
}

void TAIBufferFree(void* ptr) { tai::CommandQueue::ThreadLocal()->FreeBuffer(ptr); }*/

void* TAIGetPtr(void* buffer) { return const_cast<void*>(reinterpret_cast<const void*>(buffer)); }

void TLoad(void* src_dram_addr, uint32_t src_elem_offset, uint32_t x_size, uint32_t y_size,
           uint32_t x_stride, uint32_t x_pad_before, uint32_t y_pad_before, uint32_t x_pad_after,
           uint32_t y_pad_after, uint32_t dst_sram_index, uint32_t dst_memory_type) {
    tai::CommandQueue::ThreadLocal()->LoadBufffer(
            src_dram_addr, src_elem_offset, x_size, y_size, x_stride, x_pad_before, y_pad_before,
            x_pad_after, y_pad_after, dst_sram_index, dst_memory_type);
}

void TStore(uint32_t src_elem_offset, uint32_t src_memory_type, void* dst_dram_addr,
            uint32_t dst_elem_offset, uint32_t x_size, uint32_t y_size, uint32_t x_stride) {
    tai::CommandQueue::ThreadLocal()->StoreBuffer(src_elem_offset, src_memory_type, dst_dram_addr,
                                                  dst_elem_offset, x_size, y_size, x_stride);
}

void TAIPushMemRst(uint32_t reset_out, uint64_t dst_index, uint32_t dst_offset, uint64_t src_index,
                   uint32_t src_offset, uint64_t wgt_index, uint32_t wgt_offset) {
    tai::CommandQueue::ThreadLocal()->MemReset(reset_out, dst_index, dst_offset, src_index,
                                               src_offset, wgt_index, wgt_offset);
}

void TAIPushGemmOp(uint32_t reset_out, uint64_t dst_index, uint32_t dst_offset, uint64_t src_index,
                   uint32_t src_offset, uint64_t wgt_index, uint32_t wgt_offset) {
    tai::CommandQueue::ThreadLocal()->GemmOp(reset_out, dst_index, dst_offset, src_index, src_offset,
                                             wgt_index, wgt_offset);
}

void TAIPushCuInsts(uint32_t opcode, uint32_t extent, uint32_t reset, uint32_t dst_coeff,
                    uint32_t dst_offset, uint32_t src_coeff, uint32_t src_offset,
                    uint32_t wgt_coeff, uint32_t wgt_offset, uint32_t use_imm, int32_t imm) {
    tai::CommandQueue::ThreadLocal()->PushCuInsts(opcode, extent, reset, dst_coeff, dst_offset,
                                                  src_coeff, src_offset, wgt_coeff, wgt_offset,
                                                  use_imm, imm);
}

void TAIInitialize() {
    tai::CommandQueue::ThreadLocal()->Init();
}

void TAISynchronize() {
    tai::CommandQueue::ThreadLocal()->Synchronize();
}

static tai::Drive parseDrive(const char **str) {
	const char *s = *str;
	const char *pb = strchr(s, '#') + 1;
	const char *pe = strchr(pb, ',');
	char drive[5] = {0};
	strncpy(drive, pb, pe - pb);
    *str = pe;
	if (strcmp(drive, "NONE") == 0) {
		return tai::Drive::None;
	} else if (strcmp(drive, "INST") == 0) {
		return tai::Drive::Inst;
	} else if (strcmp(drive, "DATA") == 0) {
		return tai::Drive::Data;
	} else if (strcmp(drive, "EXU") == 0) {
		return tai::Drive::Exu;
	} else if (strcmp(drive, "MEM") == 0) {
		return tai::Drive::Mem;
	}
	return tai::Drive::None;
}

static int64_t parseHex(const char **str) {
	const char *s = *str;
    const char *pb1 = strchr(s, '#');
    const char *pb2 = strchr(s, '$');
    const char *pb = (!pb1 ? pb2 : (!pb2 ? pb1 : (pb1 < pb2 ? pb1 : pb2)));
    pb += 3;
	int64_t value = 0;
	while (*pb != '\0') {
		char c = *pb;
		if ('0' <= c && c <= '9') {
			value = value * 16 + (c - '0');
		} else if ('a' <= c && c <= 'f') {
			value = value * 16 + (c - 'a' + 10);
		} else if ('A' <= c && c <= 'F') {
			value = value * 16 + (c - 'A' + 10);
		} else {
			break;
		}
		++pb;
	}
	*str = pb;
	return value;
}

static std::string parseTarget(const char **str) {
    const char *s = *str;
    const char *pb = strchr(s, '#');
    const char *pe = strchr(pb+1, ',');
    if (!pe) pe = s + strlen(s);
    char *ret = (char *)malloc(pe - pb);
    strncpy(ret, pb+1, pe - pb - 1);
    ret[pe - pb - 1] = '\0';
    *str = pe + 1;
    return std::string(ret);
}

static uint32_t parseSpecName(const char **str) {
    const char *s = *str;
    const char *pb = strchr(s, '$');
    const char *pe = strchr(pb+1, ',');
    if (!pe) pe = s + strlen(s);
    char name[15] = {0};
    strncpy(name, pb+1, pe - pb - 1);
    *str = pe + 1;
    if (strcmp(name, "VERSION") == 0) return tai::SpecRegNames::VERSION;
    if (strcmp(name, "FFAULTS") == 0) return tai::SpecRegNames::FFAULTS;
    if (strcmp(name, "FTRIM") == 0) return tai::SpecRegNames::FTRIM;
    if (strcmp(name, "STATUS") == 0) return tai::SpecRegNames::STATUS;
    if (strcmp(name, "TASKID") == 0) return tai::SpecRegNames::TASKID;
    if (strcmp(name, "TASKCC") == 0) return tai::SpecRegNames::TASKCC;
    if (strcmp(name, "PROGH") == 0) return tai::SpecRegNames::PROGH;
    if (strcmp(name, "PROGL") == 0) return tai::SpecRegNames::PROGL;
    if (strcmp(name, "GCCH") == 0) return tai::SpecRegNames::GCCH;
    if (strcmp(name, "GCCL") == 0) return tai::SpecRegNames::GCCL;
    if (strcmp(name, "TBD") == 0) return tai::SpecRegNames::TBD;
    if (strcmp(name, "RET") == 0) return tai::SpecRegNames::RET;
    if (strcmp(name, "LWGAP") == 0) return tai::SpecRegNames::LWGAP;
    if (strcmp(name, "LWIDTH") == 0) return tai::SpecRegNames::LWIDTH;
    if (strcmp(name, "LHGAP") == 0) return tai::SpecRegNames::LHGAP;
    if (strcmp(name, "LHEIGHT") == 0) return tai::SpecRegNames::LHEIGHT;
    if (strcmp(name, "SWGAP") == 0) return tai::SpecRegNames::SWGAP;
    if (strcmp(name, "SWIDTH") == 0) return tai::SpecRegNames::SWIDTH;
    if (strcmp(name, "SHGAP") == 0) return tai::SpecRegNames::SHGAP;
    if (strcmp(name, "SHEIGHT") == 0) return tai::SpecRegNames::SHEIGHT;
    if (strcmp(name, "MSIZE") == 0) return tai::SpecRegNames::MSIZE;
    if (strcmp(name, "NSIZE") == 0) return tai::SpecRegNames::NSIZE;
    if (strcmp(name, "KSIZE") == 0) return tai::SpecRegNames::KSIZE;
    if (strcmp(name, "DWGAP") == 0) return tai::SpecRegNames::DWGAP;
    if (strcmp(name, "RWGAP") == 0) return tai::SpecRegNames::RWGAP;
    if (strcmp(name, "PEGRESS") == 0) return tai::SpecRegNames::PEGRESS;
    if (strcmp(name, "AEGRESS") == 0) return tai::SpecRegNames::AEGRESS;
    if (strcmp(name, "MEGRESS") == 0) return tai::SpecRegNames::MEGRESS;
    if (strcmp(name, "ULEN") == 0) return tai::SpecRegNames::ULEN;
    if (strcmp(name, "VLEN") == 0) return tai::SpecRegNames::VLEN;
    if (strcmp(name, "VSHIFT") == 0) return tai::SpecRegNames::VSHIFT;
    if (strcmp(name, "FWD_TMP") == 0) return tai::SpecRegNames::FWD_TMP;
    if (strcmp(name, "X_PAD_0") == 0) return tai::SpecRegNames::X_PAD_0;
    if (strcmp(name, "Y_PAD_0") == 0) return tai::SpecRegNames::Y_PAD_0;
    if (strcmp(name, "X_PAD_1") == 0) return tai::SpecRegNames::X_PAD_1;
    if (strcmp(name, "Y_PAD_1") == 0) return tai::SpecRegNames::Y_PAD_1;
    if (strcmp(name, "NDIM") == 0) return tai::SpecRegNames::NDIM;
    if (strcmp(name, "X_SIZE") == 0) return tai::SpecRegNames::X_SIZE;
    if (strcmp(name, "Y_SIZE") == 0) return tai::SpecRegNames::Y_SIZE;
    if (strcmp(name, "Z_SIZE") == 0) return tai::SpecRegNames::Z_SIZE;
    if (strcmp(name, "X_AXIS") == 0) return tai::SpecRegNames::X_AXIS;
    if (strcmp(name, "Y_AXIS") == 0) return tai::SpecRegNames::Y_AXIS;
    if (strcmp(name, "Z_AXIS") == 0) return tai::SpecRegNames::Z_AXIS;
    if (strcmp(name, "X_STRIDE") == 0) return tai::SpecRegNames::X_STRIDE;
    if (strcmp(name, "RESET_ACC") == 0) return tai::SpecRegNames::RESET_ACC;
    if (strcmp(name, "EXTENT") == 0) return tai::SpecRegNames::EXTENT;
    if (strcmp(name, "ACCUM_OFFSET") == 0) return tai::SpecRegNames::ACCUM_OFFSET;
    if (strcmp(name, "CONST_OFFSET") == 0) return tai::SpecRegNames::CONST_OFFSET;
    if (strcmp(name, "INPUT_OFFSET") == 0) return tai::SpecRegNames::INPUT_OFFSET;
    return -1;
}
static bool isAIInst(const char *op) {
    char op3[4] = {0}, op4[5] = {0};
    strncpy(op3, op, 3);
    if (strlen(op) >= 3) strncpy(op4, op, 4);
	if (strcmp(op4, "VADD") == 0 || strcmp(op4, "VSUB") == 0 || strcmp(op4, "VMUL") == 0 ||
		strcmp(op4, "VABS") == 0 || strcmp(op4, "VSQU") == 0 || strcmp(op4, "VNEG") == 0 ||
		strcmp(op4, "VEXP") == 0 || strcmp(op4, "VLOG") == 0 || strcmp(op4, "VCON") == 0 ||
		strcmp(op4, "TRAN") == 0 || strcmp(op4, "PERM") == 0 || strcmp(op4, "VLOA") == 0 ||
		strcmp(op4, "MLOA") == 0 || strcmp(op4, "TLOA") == 0 || strcmp(op4, "VSTO") == 0 ||
		strcmp(op4, "MSTO") == 0 || strcmp(op4, "TSTO") == 0 || strcmp(op4, "MCLI") == 0 ||
		strcmp(op4, "GEMM") == 0 || strcmp(op4, "CONV") == 0 || strcmp(op4, "VSUM") == 0 ||
        strcmp(op4, "VMAX") == 0 || strcmp(op4, "VMIN") == 0 || strcmp(op4, "VREC") == 0 ||
        strcmp(op3, "MMP") == 0 || strcmp(op3, "MMA") == 0 || strcmp(op3, "SMM") == 0 || 
        strcmp(op3, "MVP") == 0 || strcmp(op3, "FFT") == 0 || strcmp(op4, "IFFT") == 0 || 
        strcmp(op3, "FIR") == 0 || strcmp(op3, "DDC") == 0 || strcmp(op4, "EXTR") == 0) {
		return true;
	}
	return false;
    //
}

static bool isMovInst(const char *op) {
    char op3[4] = {0};
	strncpy(op3, op, 3);
	if (strcmp(op3, "MOV") == 0 || strcmp(op3, "XMO") == 0 || strcmp(op3, "DMO") == 0)  {
		return true;
	}
	return false;
}
static bool isBranchInst(const char *op) {
    char op3[4] = { 0 };
	strncpy(op3, op, 3);
	if (strcmp(op3, "BEQ") == 0 || strcmp(op3, "BNE") == 0 || strcmp(op3, "BLT") == 0 || strcmp(op3, "BNL") == 0) {
		return true;
	}
    return false;
}
static bool isBasicBOP(const char *op) {
    char op3[4] = {0}, op4[5] = {0};
    strncpy(op3, op, 3);
    if (strlen(op) >= 3) strncpy(op4, op, 4);
    if (strcmp(op3, "ADD") == 0 || strcmp(op3, "SUB") == 0 || strcmp(op3, "MUL") == 0 ||
        strcmp(op3, "SLT") == 0 || strcmp(op3, "SGT") == 0 || strcmp(op3, "OR") == 0 ||
        strcmp(op3, "ORI") == 0 || strcmp(op3, "AND") == 0 || strcmp(op3, "XOR") == 0 || 
        strcmp(op3, "SRL") == 0 || strcmp(op3, "SLL") == 0 || strcmp(op4, "CADD") == 0 ||
        strcmp(op4, "CSHR") == 0 || strcmp(op4, "CMIN") == 0 || strcmp(op4, "CMAX") == 0 ) {
        return true;
    }
    return false;
}

static bool isJmpInst(const char *op) {
    if (strcmp(op, "JMP") == 0 || strcmp(op, "JMPR") == 0) return true;
    return false;
}

static bool is3Addr(const char *op) {
    char op3[4] = {0}, op4[5] = {0};
    strncpy(op3, op, 3);
    strncpy(op4, op, 4);
    if (strcmp(op4, "VADD") == 0 || strcmp(op4, "VSUB") == 0 || strcmp(op4, "VMUL") == 0 ||
        strcmp(op4, "VLOA") == 0 || strcmp(op4, "MLOA") == 0 || strcmp(op4, "TLOA") == 0 || 
        strcmp(op4, "VSTO") == 0 || strcmp(op4, "MSTO") == 0 || strcmp(op4, "TSTO") == 0 || 
        strcmp(op4, "MCLI") == 0 || strcmp(op4, "GEMM") == 0 || strcmp(op4, "CONV") == 0 ||
        strcmp(op3, "MMP") == 0 || strcmp(op3, "MMA") == 0 || strcmp(op3, "SMM") == 0 ||
        strcmp(op3, "MVP") == 0) {
		return true;
    }
    return false;
}

static void getOpcode(const char **inst, char *op) {
    const char *str = *inst;
    const char *ptr = strchr(str, ' ');
    if (!ptr) ptr = strchr(str, '\t');
    if (!ptr) {
        strcpy(op, str);
        *inst = str + strlen(str);
        return;
    }
    strncpy(op, str, ptr - str);
    *inst = ptr;
}

void TAIPushInst(const char* inst) {
    const char *str = inst;
	tai::Drive dri, dro;
	int64_t rd, rs0, rs1;
	int path;
    char op[20] = {0};
    getOpcode(&inst, op);
    tai::Instruction *ret = NULL;
	
	if (isAIInst(op)) {
		path = parseHex(&inst);
		dri = parseDrive(&inst);
		dro = parseDrive(&inst);
		rd = parseHex(&inst);
        rs0 = parseHex(&inst);
        if (is3Addr(op)) rs1 = parseHex(&inst);

        if (strcmp(op, "VADD.I32") == 0) 
            ret = tai::CommandQueue::ThreadLocal()->GetProgram()->VaddI32(path, dri, dro, (uint32_t)rd, (uint32_t)rs0, (uint32_t)rs1);
        else if (strcmp(op, "VADD.F32") == 0)
            ret = tai::CommandQueue::ThreadLocal()->GetProgram()->VaddF32(path, dri, dro, (uint32_t)rd, (uint32_t)rs0, (uint32_t)rs1);
        else if (strcmp(op, "VADD.F64") == 0)
            ret = tai::CommandQueue::ThreadLocal()->GetProgram()->VaddF64(path, dri, dro, (uint32_t)rd, (uint32_t)rs0, (uint32_t)rs1);
        else if (strcmp(op, "VADDI.I32") == 0)
            ret = tai::CommandQueue::ThreadLocal()->GetProgram()->VaddiI32(path, dri, dro, (uint32_t)rd, (uint32_t)rs0, *((int32_t *)(&rs1)));
        else if (strcmp(op, "VADDI.F32") == 0)
            ret = tai::CommandQueue::ThreadLocal()->GetProgram()->VaddiF32(path, dri, dro, (uint32_t)rd, (uint32_t)rs0, *((float *)(&rs1)));
        else if (strcmp(op, "VADDI.F64") == 0)
            ret = tai::CommandQueue::ThreadLocal()->GetProgram()->VaddiF64(path, dri, dro, (uint32_t)rd, (uint32_t)rs0, *((double *)(&rs1)));
        else if (strcmp(op, "VSUB.I32") == 0) 
            ret = tai::CommandQueue::ThreadLocal()->GetProgram()->VsubI32(path, dri, dro, (uint32_t)rd, (uint32_t)rs0, (uint32_t)rs1);
        else if (strcmp(op, "VSUB.F32") == 0)
            ret = tai::CommandQueue::ThreadLocal()->GetProgram()->VsubF32(path, dri, dro, (uint32_t)rd, (uint32_t)rs0, (uint32_t)rs1);
        else if (strcmp(op, "VSUB.F64") == 0)
            ret = tai::CommandQueue::ThreadLocal()->GetProgram()->VsubF64(path, dri, dro, (uint32_t)rd, (uint32_t)rs0, (uint32_t)rs1);
        else if (strcmp(op, "VSUB.C32") == 0)
            ret = tai::CommandQueue::ThreadLocal()->GetProgram()->VsubC32(path, dri, dro, (uint32_t)rd, (uint32_t)rs0, (uint32_t)rs1);
        else if (strcmp(op, "VSUB.C64") == 0)
            ret = tai::CommandQueue::ThreadLocal()->GetProgram()->VsubC64(path, dri, dro, (uint32_t)rd, (uint32_t)rs0, (uint32_t)rs1);
        else if (strcmp(op, "VSUBI.I32") == 0)
            ret = tai::CommandQueue::ThreadLocal()->GetProgram()->VsubiI32(path, dri, dro, (uint32_t)rd, (uint32_t)rs0, *((int32_t *)(&rs1)));
        else if (strcmp(op, "VSUBI.F32") == 0)
            ret = tai::CommandQueue::ThreadLocal()->GetProgram()->VsubiF32(path, dri, dro, (uint32_t)rd, (uint32_t)rs0, *((float *)(&rs1)));
        else if (strcmp(op, "VSUBI.F64") == 0)
            ret = tai::CommandQueue::ThreadLocal()->GetProgram()->VsubiF64(path, dri, dro, (uint32_t)rd, (uint32_t)rs0, *((double *)(&rs1)));
        else if (strcmp(op, "VMUL.I32") == 0) 
            ret = tai::CommandQueue::ThreadLocal()->GetProgram()->VmulI32(path, dri, dro, (uint32_t)rd, (uint32_t)rs0, (uint32_t)rs1);
        else if (strcmp(op, "VMUL.F32") == 0)
            ret = tai::CommandQueue::ThreadLocal()->GetProgram()->VmulF32(path, dri, dro, (uint32_t)rd, (uint32_t)rs0, (uint32_t)rs1);
        else if (strcmp(op, "VMUL.F64") == 0)
            ret = tai::CommandQueue::ThreadLocal()->GetProgram()->VmulF64(path, dri, dro, (uint32_t)rd, (uint32_t)rs0, (uint32_t)rs1);
        else if (strcmp(op, "VMUL.C32") == 0)
            ret = tai::CommandQueue::ThreadLocal()->GetProgram()->VmulC32(path, dri, dro, (uint32_t)rd, (uint32_t)rs0, (uint32_t)rs1);
        else if (strcmp(op, "VMULI.I32") == 0)
            ret = tai::CommandQueue::ThreadLocal()->GetProgram()->VmuliI32(path, dri, dro, (uint32_t)rd, (uint32_t)rs0, *((int32_t *)(&rs1)));
        else if (strcmp(op, "VMULI.F32") == 0)
            ret = tai::CommandQueue::ThreadLocal()->GetProgram()->VmuliF32(path, dri, dro, (uint32_t)rd, (uint32_t)rs0, *((float *)(&rs1)));
        else if (strcmp(op, "VMULI.F64") == 0)
            ret = tai::CommandQueue::ThreadLocal()->GetProgram()->VmuliF64(path, dri, dro, (uint32_t)rd, (uint32_t)rs0, *((double *)(&rs1)));
        else if (strcmp(op, "VMULI.C32") == 0) 
            ret = tai::CommandQueue::ThreadLocal()->GetProgram()->VmuliC32(path, dri, dro, (uint32_t)rd, (uint32_t)rs0, *((float _Complex*)(&rs1)));
        else if (strcmp(op, "VMULI.C64") == 0) {
            int64_t imag = parseHex(&inst);
            double _Complex imm = *((double *)(&rs1)) + *((double *)(&imag)) * I;
            ret = tai::CommandQueue::ThreadLocal()->GetProgram()->VmuliC64(path, dri, dro, (uint32_t)rd, (uint32_t)rs0, imm);
        }
        else if (strcmp(op, "VABS.I32") == 0)
            ret = tai::CommandQueue::ThreadLocal()->GetProgram()->VabsI32(path, dri, dro, (uint32_t)rd, (uint32_t)rs0);
        else if (strcmp(op, "VABS.F32") == 0)
            ret = tai::CommandQueue::ThreadLocal()->GetProgram()->VabsF32(path, dri, dro, (uint32_t)rd, (uint32_t)rs0);  
        else if (strcmp(op, "VABS.F64") == 0)
            ret = tai::CommandQueue::ThreadLocal()->GetProgram()->VabsF64(path, dri, dro, (uint32_t)rd, (uint32_t)rs0);   
        else if (strcmp(op, "VABS.C32") == 0)
            ret = tai::CommandQueue::ThreadLocal()->GetProgram()->VabsC32(path, dri, dro, (uint32_t)rd, (uint32_t)rs0);  
        else if (strcmp(op, "VABS.C64") == 0)
            ret = tai::CommandQueue::ThreadLocal()->GetProgram()->VabsC64(path, dri, dro, (uint32_t)rd, (uint32_t)rs0);   
        else if (strcmp(op, "VSQUA.I32") == 0)
            ret = tai::CommandQueue::ThreadLocal()->GetProgram()->VsquaI32(path, dri, dro, (uint32_t)rd, (uint32_t)rs0);
        else if (strcmp(op, "VSQUA.F32") == 0)
            ret = tai::CommandQueue::ThreadLocal()->GetProgram()->VsquaF32(path, dri, dro, (uint32_t)rd, (uint32_t)rs0);
        else if (strcmp(op, "VSQUA.F64") == 0)
            ret = tai::CommandQueue::ThreadLocal()->GetProgram()->VsquaF64(path, dri, dro, (uint32_t)rd, (uint32_t)rs0);
        else if (strcmp(op, "VNEG.I32") == 0)
            ret = tai::CommandQueue::ThreadLocal()->GetProgram()->VnegI32(path, dri, dro, (uint32_t)rd, (uint32_t)rs0);
        else if (strcmp(op, "VNEG.F32") == 0)
            ret = tai::CommandQueue::ThreadLocal()->GetProgram()->VnegF32(path, dri, dro, (uint32_t)rd, (uint32_t)rs0);
        else if (strcmp(op, "VNEG.F64") == 0)
            ret = tai::CommandQueue::ThreadLocal()->GetProgram()->VnegF64(path, dri, dro, (uint32_t)rd, (uint32_t)rs0);
        else if (strcmp(op, "VREC.I32") == 0)
            ret = tai::CommandQueue::ThreadLocal()->GetProgram()->VrecI32(path, dri, dro, (uint32_t)rd, (uint32_t)rs0);
        else if (strcmp(op, "VREC.F32") == 0)
            ret = tai::CommandQueue::ThreadLocal()->GetProgram()->VrecF32(path, dri, dro, (uint32_t)rd, (uint32_t)rs0);
        else if (strcmp(op, "VREC.F64") == 0)
            ret = tai::CommandQueue::ThreadLocal()->GetProgram()->VrecF64(path, dri, dro, (uint32_t)rd, (uint32_t)rs0);        
        else if (strcmp(op, "VEXP.I32") == 0)
            ret = tai::CommandQueue::ThreadLocal()->GetProgram()->VexpI32(path, dri, dro, (uint32_t)rd, (uint32_t)rs0);
        else if (strcmp(op, "VEXP.F32") == 0)
            ret = tai::CommandQueue::ThreadLocal()->GetProgram()->VexpF32(path, dri, dro, (uint32_t)rd, (uint32_t)rs0);
        else if (strcmp(op, "VEXP.F64") == 0)
            ret = tai::CommandQueue::ThreadLocal()->GetProgram()->VexpF64(path, dri, dro, (uint32_t)rd, (uint32_t)rs0);
        else if (strcmp(op, "VLOG10.I32") == 0)
            ret = tai::CommandQueue::ThreadLocal()->GetProgram()->Vlog10I32(path, dri, dro, (uint32_t)rd, (uint32_t)rs0);
        else if (strcmp(op, "VLOG10.F32") == 0)
            ret = tai::CommandQueue::ThreadLocal()->GetProgram()->Vlog10F32(path, dri, dro, (uint32_t)rd, (uint32_t)rs0);
        else if (strcmp(op, "VLOG10.F64") == 0)
            ret = tai::CommandQueue::ThreadLocal()->GetProgram()->Vlog10F64(path, dri, dro, (uint32_t)rd, (uint32_t)rs0);
        else if (strcmp(op, "VCONJ.C32") == 0)
            ret = tai::CommandQueue::ThreadLocal()->GetProgram()->VconjC32(path, dri, dro, (uint32_t)rd, (uint32_t)rs0);
        else if (strcmp(op, "VCONJ.C64") == 0)
            ret = tai::CommandQueue::ThreadLocal()->GetProgram()->VconjC64(path, dri, dro, (uint32_t)rd, (uint32_t)rs0);
        else if (strcmp(op, "VSUM.I32") == 0)
            ret = tai::CommandQueue::ThreadLocal()->GetProgram()->VsumI32(path, dri, dro, (uint32_t)rd, (uint32_t)rs0);
        else if (strcmp(op, "VSUM.F32") == 0)
            ret = tai::CommandQueue::ThreadLocal()->GetProgram()->VsumF32(path, dri, dro, (uint32_t)rd, (uint32_t)rs0);
        else if (strcmp(op, "VSUM.F64") == 0)
            ret = tai::CommandQueue::ThreadLocal()->GetProgram()->VsumF64(path, dri, dro, (uint32_t)rd, (uint32_t)rs0);
        else if (strcmp(op, "VMAX.I32") == 0)
            ret = tai::CommandQueue::ThreadLocal()->GetProgram()->VmaxI32(path, dri, dro, (uint32_t)rd, (uint32_t)rs0);
        else if (strcmp(op, "VMAX.F32") == 0)
            ret = tai::CommandQueue::ThreadLocal()->GetProgram()->VmaxF32(path, dri, dro, (uint32_t)rd, (uint32_t)rs0);
        else if (strcmp(op, "VMAX.F64") == 0)
            ret = tai::CommandQueue::ThreadLocal()->GetProgram()->VmaxF64(path, dri, dro, (uint32_t)rd, (uint32_t)rs0);  
        else if (strcmp(op, "VMIN.I32") == 0)
            ret = tai::CommandQueue::ThreadLocal()->GetProgram()->VminI32(path, dri, dro, (uint32_t)rd, (uint32_t)rs0);
        else if (strcmp(op, "VMIN.F32") == 0)
            ret = tai::CommandQueue::ThreadLocal()->GetProgram()->VminF32(path, dri, dro, (uint32_t)rd, (uint32_t)rs0);
        else if (strcmp(op, "VMIN.F64") == 0)
            ret = tai::CommandQueue::ThreadLocal()->GetProgram()->VminF64(path, dri, dro, (uint32_t)rd, (uint32_t)rs0);  
        else if (strcmp(op, "TRANSPOSE.I32") == 0)
            ret = tai::CommandQueue::ThreadLocal()->GetProgram()->TransposeI32(path, dri, dro, (uint32_t)rd, (uint32_t)rs0);
        else if (strcmp(op, "TRANSPOSE.F32") == 0)
            ret = tai::CommandQueue::ThreadLocal()->GetProgram()->TransposeF32(path, dri, dro, (uint32_t)rd, (uint32_t)rs0);
        else if (strcmp(op, "TRANSPOSE.F64") == 0)
            ret = tai::CommandQueue::ThreadLocal()->GetProgram()->TransposeF64(path, dri, dro, (uint32_t)rd, (uint32_t)rs0);   
        else if (strcmp(op, "PERMUTE.I32") == 0)
            ret = tai::CommandQueue::ThreadLocal()->GetProgram()->PermuteI32(path, dri, dro, (uint32_t)rd, (uint32_t)rs0);
        else if (strcmp(op, "PERMUTE.F32") == 0)
            ret = tai::CommandQueue::ThreadLocal()->GetProgram()->PermuteF32(path, dri, dro, (uint32_t)rd, (uint32_t)rs0);
        else if (strcmp(op, "PERMUTE.F64") == 0)
            ret = tai::CommandQueue::ThreadLocal()->GetProgram()->PermuteF64(path, dri, dro, (uint32_t)rd, (uint32_t)rs0); 
        else if (strcmp(op, "VLOAD") == 0)
            ret = tai::CommandQueue::ThreadLocal()->GetProgram()->Vload(path, dri, dro, (uint32_t)rd, (uint32_t)rs0, (uint32_t)rs1);
        else if (strcmp(op, "MLOAD") == 0)
            ret = tai::CommandQueue::ThreadLocal()->GetProgram()->Mload(path, dri, dro, (uint32_t)rd, (uint32_t)rs0, (uint32_t)rs1);
        else if (strcmp(op, "TLOAD") == 0)
            ret = tai::CommandQueue::ThreadLocal()->GetProgram()->Tload(path, dri, dro, (uint32_t)rd, (uint32_t)rs0, (uint32_t)rs1);
        else if (strcmp(op, "VSTORE") == 0)
            ret = tai::CommandQueue::ThreadLocal()->GetProgram()->Vstore(path, dri, dro, (uint32_t)rd, (uint32_t)rs0, (uint32_t)rs1);
        else if (strcmp(op, "MSTORE") == 0)
            ret = tai::CommandQueue::ThreadLocal()->GetProgram()->Mstore(path, dri, dro, (uint32_t)rd, (uint32_t)rs0, (uint32_t)rs1);       
        else if (strcmp(op, "TSTORE") == 0)
            ret = tai::CommandQueue::ThreadLocal()->GetProgram()->Tstore(path, dri, dro, (uint32_t)rd, (uint32_t)rs0, (uint32_t)rs1);
        else if (strcmp(op, "MMP") == 0)
            ret = tai::CommandQueue::ThreadLocal()->GetProgram()->Mmp(path, dri, dro, (uint32_t)rd, (uint32_t)rs0, (uint32_t)rs1);
        else if (strcmp(op, "MMA") == 0)
            ret = tai::CommandQueue::ThreadLocal()->GetProgram()->Mma(path, dri, dro, (uint32_t)rd, (uint32_t)rs0, (uint32_t)rs1);
        else if (strcmp(op, "SMM") == 0)
            ret = tai::CommandQueue::ThreadLocal()->GetProgram()->Smm(path, dri, dro, (uint32_t)rd, (uint32_t)rs0, (uint32_t)rs1);
        else if (strcmp(op, "MCLIP") == 0)
            ret = tai::CommandQueue::ThreadLocal()->GetProgram()->Mclip(path, dri, dro, (uint32_t)rd, (uint32_t)rs0, (uint32_t)rs1);
        else if (strcmp(op, "MVP") == 0)
            ret = tai::CommandQueue::ThreadLocal()->GetProgram()->Mvp(path, dri, dro, (uint32_t)rd, (uint32_t)rs0, (uint32_t)rs1);
        else if (strcmp(op, "GEMM.I32") == 0)
            ret = tai::CommandQueue::ThreadLocal()->GetProgram()->GemmI32(path, dri, dro, (uint32_t)rd, (uint32_t)rs0, (uint32_t)rs1);
        else if (strcmp(op, "GEMM.F32") == 0)
            ret = tai::CommandQueue::ThreadLocal()->GetProgram()->GemmF32(path, dri, dro, (uint32_t)rd, (uint32_t)rs0, (uint32_t)rs1);
        else if (strcmp(op, "GEMM.F64") == 0)
            ret = tai::CommandQueue::ThreadLocal()->GetProgram()->GemmF64(path, dri, dro, (uint32_t)rd, (uint32_t)rs0, (uint32_t)rs1);
        else if (strcmp(op, "GEMM.C32") == 0)
            ret = tai::CommandQueue::ThreadLocal()->GetProgram()->GemmC32(path, dri, dro, (uint32_t)rd, (uint32_t)rs0, (uint32_t)rs1);
        else if (strcmp(op, "GEMM.C64") == 0)
            ret = tai::CommandQueue::ThreadLocal()->GetProgram()->GemmC64(path, dri, dro, (uint32_t)rd, (uint32_t)rs0, (uint32_t)rs1);
        else if (strcmp(op, "CONV") == 0)
            ret = tai::CommandQueue::ThreadLocal()->GetProgram()->Conv(path, dri, dro, (uint32_t)rd, (uint32_t)rs0, (uint32_t)rs1);
        else if (strcmp(op, "FFT") == 0)
            ret = tai::CommandQueue::ThreadLocal()->GetProgram()->Fft(path, dri, dro, (uint32_t)rd, (uint32_t)rs0);
        else if (strcmp(op, "IFFT") == 0)
            ret = tai::CommandQueue::ThreadLocal()->GetProgram()->Ifft(path, dri, dro, (uint32_t)rd, (uint32_t)rs0);
        else if (strcmp(op, "FIR") == 0)
            ret = tai::CommandQueue::ThreadLocal()->GetProgram()->Fir(path, dri, dro, (uint32_t)rd, (uint32_t)rs0, (uint32_t)rs1);
        else if (strcmp(op, "EXTR") == 0)
        ret = tai::CommandQueue::ThreadLocal()->GetProgram()->Extr(path, dri, dro, (uint32_t)rd, (uint32_t)rs0);
        else if (strcmp(op, "DDC") == 0)
            ret = tai::CommandQueue::ThreadLocal()->GetProgram()->Ddc(path, dri, dro, (uint32_t)rd, (uint32_t)rs0);
    
} else if (isMovInst(op)) {
        if (strcmp(op, "MOVID") == 0 || strcmp(op, "DMOVO") == 0) {
            rd = parseSpecName(&inst);
            rs0 = parseHex(&inst);
            if (strcmp(op, "MOVID") == 0)
                ret = tai::CommandQueue::ThreadLocal()->GetProgram()->Movid((uint32_t)rd, rs0);
            else
                ret = tai::CommandQueue::ThreadLocal()->GetProgram()->Dmovo((uint32_t)rd, (uint32_t)rs0);
        } else if (strcmp(op, "DMOVI") == 0) {
            rd = parseHex(&inst);
            rs0 = parseSpecName(&inst);
            ret = tai::CommandQueue::ThreadLocal()->GetProgram()->Dmovi((uint32_t)rd, (uint32_t)rs0);
        } else {  
            rd = parseHex(&inst);
            rs0 = parseHex(&inst);
            
            if (strcmp(op, "MOV") == 0)
                ret = tai::CommandQueue::ThreadLocal()->GetProgram()->Mov((uint32_t)rd, (uint32_t)rs0);
            else if (strcmp(op, "MOVI") == 0) 
                ret = tai::CommandQueue::ThreadLocal()->GetProgram()->Movi((uint32_t)rd, rs0);
            else if (strcmp(op, "XMOVI") == 0) 
                ret = tai::CommandQueue::ThreadLocal()->GetProgram()->Xmovi((uint32_t)rd, (uint32_t)rs0);
            else
                ret = tai::CommandQueue::ThreadLocal()->GetProgram()->Xmovo((uint32_t)rd, (uint32_t)rs0);
        }
    } else if (isBranchInst(op)) {
        const std::string &target = parseTarget(&inst);
		rd = parseHex(&inst);
        rs0 = parseHex(&inst);

		if (strcmp(op, "BEQ") == 0)
			ret = tai::CommandQueue::ThreadLocal()->GetProgram()->Beq(target, (uint32_t)rd, (uint32_t)rs0);
		else if (strcmp(op, "BEQI") == 0)
			ret = tai::CommandQueue::ThreadLocal()->GetProgram()->Beqi(target, (uint32_t)rd, (int32_t)rs0);
		else if (strcmp(op, "BNE") == 0)
			ret = tai::CommandQueue::ThreadLocal()->GetProgram()->Bne(target, (uint32_t)rd, (uint32_t)rs0);
		else if (strcmp(op, "BNEI") == 0)
			ret = tai::CommandQueue::ThreadLocal()->GetProgram()->Bnei(target, (uint32_t)rd, (int32_t)rs0);
		else if (strcmp(op, "BLT") == 0)
			ret = tai::CommandQueue::ThreadLocal()->GetProgram()->Blt(target, (uint32_t)rd, (uint32_t)rs0);
		else if (strcmp(op, "BLTI") == 0)
			ret = tai::CommandQueue::ThreadLocal()->GetProgram()->Blti(target, (uint32_t)rd, (int32_t)rs0);
		else if (strcmp(op, "BNL") == 0)
			ret = tai::CommandQueue::ThreadLocal()->GetProgram()->Bnl(target, (uint32_t)rd, (uint32_t)rs0);
		else
			ret = tai::CommandQueue::ThreadLocal()->GetProgram()->Bnli(target, (uint32_t)rd, (int32_t)rs0);
    } else if (isBasicBOP(op)) {
        rd = parseHex(&inst);
        rs0 = parseHex(&inst);
        rs1 = parseHex(&inst);

        if (strcmp(op, "ADD") == 0) 
            ret = tai::CommandQueue::ThreadLocal()->GetProgram()->Add((uint32_t)rd, (uint32_t)rs0, (uint32_t)rs1);
        else if (strcmp(op, "ADDI") == 0)
            ret = tai::CommandQueue::ThreadLocal()->GetProgram()->Addi((uint32_t)rd, (uint32_t)rs0, (int)rs1);
        else if (strcmp(op, "SUB") == 0)
            ret = tai::CommandQueue::ThreadLocal()->GetProgram()->Sub((uint32_t)rd, (uint32_t)rs0, (uint32_t)rs1);
        else if (strcmp(op, "SUBI") == 0)
            ret = tai::CommandQueue::ThreadLocal()->GetProgram()->Subi((uint32_t)rd, (uint32_t)rs0, (int)rs1);
        else if (strcmp(op, "MUL") == 0)
            ret = tai::CommandQueue::ThreadLocal()->GetProgram()->Mul((uint32_t)rd, (uint32_t)rs0, (uint32_t)rs1);
        else if (strcmp(op, "MULI") == 0)
            ret = tai::CommandQueue::ThreadLocal()->GetProgram()->Muli((uint32_t)rd, (uint32_t)rs0, (int)rs1);
        else if (strcmp(op, "SLT") == 0)
            ret = tai::CommandQueue::ThreadLocal()->GetProgram()->Slt((uint32_t)rd, (uint32_t)rs0, (uint32_t)rs1);
        else if (strcmp(op, "SLTI") == 0)
            ret = tai::CommandQueue::ThreadLocal()->GetProgram()->Slti((uint32_t)rd, (uint32_t)rs0, (int)rs1);
        else if (strcmp(op, "SGT") == 0)
            ret = tai::CommandQueue::ThreadLocal()->GetProgram()->Sgt((uint32_t)rd, (uint32_t)rs0, (uint32_t)rs1);
        else if (strcmp(op, "SGTI") == 0)
            ret = tai::CommandQueue::ThreadLocal()->GetProgram()->Sgti((uint32_t)rd, (uint32_t)rs0, (int)rs1);
        else if (strcmp(op, "OR") == 0)
            ret = tai::CommandQueue::ThreadLocal()->GetProgram()->Or((uint32_t)rd, (uint32_t)rs0, (uint32_t)rs1);
        else if (strcmp(op, "ORI") == 0)
            ret = tai::CommandQueue::ThreadLocal()->GetProgram()->Ori((uint32_t)rd, (uint32_t)rs0, (int)rs1);  
        else if (strcmp(op, "AND") == 0)
            ret = tai::CommandQueue::ThreadLocal()->GetProgram()->And((uint32_t)rd, (uint32_t)rs0, (uint32_t)rs1);
        else if (strcmp(op, "ANDI") == 0)
            ret = tai::CommandQueue::ThreadLocal()->GetProgram()->Andi((uint32_t)rd, (uint32_t)rs0, (int)rs1);
        else if (strcmp(op, "XOR") == 0)
            ret = tai::CommandQueue::ThreadLocal()->GetProgram()->Xor((uint32_t)rd, (uint32_t)rs0, (uint32_t)rs1);
        else if (strcmp(op, "XORI") == 0)
            ret = tai::CommandQueue::ThreadLocal()->GetProgram()->Xori((uint32_t)rd, (uint32_t)rs0, (int)rs1);
        else if (strcmp(op, "SRL") == 0)
            ret = tai::CommandQueue::ThreadLocal()->GetProgram()->Srl((uint32_t)rd, (uint32_t)rs0, (uint32_t)rs1);
        else if (strcmp(op, "SRLI") == 0)
            ret = tai::CommandQueue::ThreadLocal()->GetProgram()->Srli((uint32_t)rd, (uint32_t)rs0, (int)rs1);
        else if (strcmp(op, "SLL") == 0)
            ret = tai::CommandQueue::ThreadLocal()->GetProgram()->Sll((uint32_t)rd, (uint32_t)rs0, (uint32_t)rs1);
        else if (strcmp(op, "SLLI") == 0)
            ret = tai::CommandQueue::ThreadLocal()->GetProgram()->Slli((uint32_t)rd, (uint32_t)rs0, (int)rs1);
        else if (strcmp(op, "CADD") == 0)
            ret = tai::CommandQueue::ThreadLocal()->GetProgram()->cAdd((uint32_t)rd, (uint32_t)rs0, (uint32_t)rs1);
        else if (strcmp(op, "CADDI") == 0)
            ret = tai::CommandQueue::ThreadLocal()->GetProgram()->cAddi((uint32_t)rd, (uint32_t)rs0, (int)rs1);
        else if (strcmp(op, "CSHR") == 0)
            ret = tai::CommandQueue::ThreadLocal()->GetProgram()->cShr((uint32_t)rd, (uint32_t)rs0, (uint32_t)rs1);
        else if (strcmp(op, "CSHRI") == 0)
            ret = tai::CommandQueue::ThreadLocal()->GetProgram()->cShri((uint32_t)rd, (uint32_t)rs0, (int)rs1);
        else if (strcmp(op, "CMIN") == 0)
            ret = tai::CommandQueue::ThreadLocal()->GetProgram()->cMin((uint32_t)rd, (uint32_t)rs0, (uint32_t)rs1);
        else if (strcmp(op, "CMINI") == 0)
            ret = tai::CommandQueue::ThreadLocal()->GetProgram()->cMini((uint32_t)rd, (uint32_t)rs0, (int)rs1);
        else if (strcmp(op, "CMAX") == 0)
            ret = tai::CommandQueue::ThreadLocal()->GetProgram()->cMax((uint32_t)rd, (uint32_t)rs0, (uint32_t)rs1);
        else
            ret = tai::CommandQueue::ThreadLocal()->GetProgram()->cMaxi((uint32_t)rd, (uint32_t)rs0, (int)rs1);
    } else if (isJmpInst(op)) {
        rd = parseHex(&inst);
        if (strcmp(op, "JMP") == 0) {
            const std::string &target = parseTarget(&inst);
            ret = tai::CommandQueue::ThreadLocal()->GetProgram()->Jmp(rd, target);
        } else {
            rs0 = parseHex(&inst);
            rs1 = parseHex(&inst);
            ret = tai::CommandQueue::ThreadLocal()->GetProgram()->Jmpr((uint32_t)rd, (uint32_t)rs0, (int)rs1);
        }
    } else {
        if (strcmp(op, "MEMSET") == 0) {
            rd = parseHex(&inst);
            rs0 = parseHex(&inst);
            rs1 = parseHex(&inst);
            ret = tai::CommandQueue::ThreadLocal()->GetProgram()->MemSet((uint32_t)rd, (uint32_t)rs0, (uint32_t)rs1);
        } else if (strcmp(op, "CALL") == 0) {
            const std::string &target = parseTarget(&inst);
            const std::string &dev = parseTarget(&inst);
            rd = parseHex(&inst);
            rs0 = parseHex(&inst);
            rs1 = parseHex(&inst);
            ret = tai::CommandQueue::ThreadLocal()->GetProgram()->Call(target, dev, (int)rd, (int)rs0, (int)rs1);
        } else if (strcmp(op, "DISPLAY") == 0) {
            const char *pb = strchr(inst, '\"');
            const char *pe = strchr(pb+1, '\"');
            char *str = (char *)malloc(pe - pb);
            strncpy(str, pb+1, pe - pb - 1);
            str[pe - pb - 1] = '\0';
            inst = pe + 1;
            rd = parseHex(&inst);
            ret = tai::CommandQueue::ThreadLocal()->GetProgram()->Display(str, (uint32_t)rd);
        } else if (strcmp(op, "FENCE") == 0) {
            rd = parseHex(&inst);
            ret = tai::CommandQueue::ThreadLocal()->GetProgram()->Fence((uint32_t)rd);
        } else if (strcmp(op, "RET") == 0) {
            ret = tai::CommandQueue::ThreadLocal()->GetProgram()->Ret();
        } else if (strcmp(op, "HALT") == 0) {
            ret = tai::CommandQueue::ThreadLocal()->GetProgram()->Halt();
        }
    }
    tai::CommandQueue::ThreadLocal()->PushInst(ret);
}
