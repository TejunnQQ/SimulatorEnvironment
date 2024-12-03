#ifndef TAI_SIM_TAI_INST_H
#define TAI_SIM_TAI_INST_H

#include <set>
#include <map>
#include <list>
#include <functional>
#include "tai_spec.h"

namespace tai {
    enum class Type {
        Label,
        BasicInst,
        AiInst,
    };

    enum class Tag {
        None,
        Call,
        Ret,
        Fence,
        Load,
        Store,
        MatCompute,
        VecCompute,
    };

    enum class Drive {
        None,
        Inst,
        Data,
        Exu,
        Mem,
    };

    struct Unit;

    struct Instruction {
        Instruction() = default;
        virtual ~Instruction() = default;
        std::string name;
        Type type_;
        Tag tag_;
        uint32_t rd_;
        uint32_t rs0_;
        uint32_t rs1_;
        std::function<void(Unit*)> kernel_;
    };

    struct Label : Instruction {
        Label(const std::string& t);
        ~Label() override = default;
        std::string text_;
    };

    struct BasicInst : Instruction {
        BasicInst(std::function<void(Unit*)> k);
        ~BasicInst() override = default;
    };

    struct AiInst : Instruction {
        AiInst(std::function<void(Unit*)> k, Tag t = Tag::None);
        AiInst(int p, Drive dri, Drive dro, std::function<void(Unit*)> k, Tag t = Tag::None);
        ~AiInst() override = default;

        int path_ = 0;
        Drive driver_;
        Drive driven_;
    };

    struct Program {
        Program();
        ~Program();

        void CreateFunc(const std::string& name, std::vector<Instruction*> ivec);
        Instruction* CreateLabel(const std::string& l);
        std::string GetLabel(const std::string& l);
        void Clear();

        Instruction* Jmp(uint32_t rd, const std::string& target);
        Instruction* Jmpr(uint32_t rd, uint32_t rs0, int offset);
        Instruction* Beq(const std::string& target, uint32_t rs0, uint32_t rs1);
        Instruction* Beqi(const std::string& target, uint32_t rs0, int32_t imm);
        Instruction* Bne(const std::string& target, uint32_t rs0, uint32_t rs1);
        Instruction* Bnei(const std::string& target, uint32_t rs0, int32_t imm);
        Instruction* Blt(const std::string& target, uint32_t rs0, uint32_t rs1);
        Instruction* Blti(const std::string& target, uint32_t rs0, int32_t imm);
        Instruction* Bnl(const std::string& target, uint32_t rs0, uint32_t rs1);
        Instruction* Bnli(const std::string& target, uint32_t rs0, int32_t imm);

        Instruction* Mov(uint32_t rd, uint32_t rs0);
        Instruction* Movi(uint32_t rd, int64_t imm);
        Instruction* Movid(uint32_t drd, int64_t imm);
        Instruction* Xmovi(uint32_t rd, uint32_t rs0);
        Instruction* Xmovo(uint32_t rd, uint32_t rs0);
        Instruction* Dmovi(uint32_t rd, uint32_t drs0);
        Instruction* Dmovo(uint32_t drd, uint32_t rs0);

        Instruction* Add(uint32_t rd, uint32_t rs0, uint32_t rs1);
        Instruction* Addi(uint32_t rd, uint32_t rs0, int imm);
        Instruction* Sub(uint32_t rd, uint32_t rs0, uint32_t rs1);
        Instruction* Subi(uint32_t rd, uint32_t rs0, int imm);
        Instruction* Mul(uint32_t rd, uint32_t rs0, uint32_t rs1);
        Instruction* Muli(uint32_t rd, uint32_t rs0, int imm);
        Instruction* Slt(uint32_t rd, uint32_t rs0, uint32_t rs1);
        Instruction* Slti(uint32_t rd, uint32_t rs0, int imm);
        Instruction* Sgt(uint32_t rd, uint32_t rs0, uint32_t rs1);
        Instruction* Sgti(uint32_t rd, uint32_t rs0, int imm);
        Instruction* Or(uint32_t rd, uint32_t rs0, uint32_t rs1);
        Instruction* Ori(uint32_t rd, uint32_t rs0, int imm);
        Instruction* And(uint32_t rd, uint32_t rs0, uint32_t rs1);
        Instruction* Andi(uint32_t rd, uint32_t rs0, int imm);
        Instruction* Xor(uint32_t rd, uint32_t rs0, uint32_t rs1);
        Instruction* Xori(uint32_t rd, uint32_t rs0, int imm);
        Instruction* Srl(uint32_t rd, uint32_t rs0, uint32_t rs1);
        Instruction* Srli(uint32_t rd, uint32_t rs0, int imm);
        Instruction* Sll(uint32_t rd, uint32_t rs0, uint32_t rs1);
        Instruction* Slli(uint32_t rd, uint32_t rs0, int imm);
        // rd is dst addr, rs0 is src addr
        Instruction* cAddi(uint32_t rd, uint32_t rs0, int32_t imm);
        Instruction* cAdd(uint32_t rd, uint32_t rs0, uint32_t rs1);
        Instruction* cShri(uint32_t rd, uint32_t rs0, int32_t imm);
        Instruction* cShr(uint32_t rd, uint32_t rs0, uint32_t rs1);
        Instruction* cMini(uint32_t rd, uint32_t rs0, int32_t imm);
        Instruction* cMin(uint32_t rd, uint32_t rs0, uint32_t rs1);
        Instruction* cMaxi(uint32_t rd, uint32_t rs0, int32_t imm);
        Instruction* cMax(uint32_t rd, uint32_t rs0, uint32_t rs1);

        // vector element-wise operations
        // add (vlen)
        Instruction *VaddI32(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs0, uint32_t rs1);
        Instruction *VaddF32(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs0, uint32_t rs1);
        Instruction *VaddF64(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs0, uint32_t rs1);
        Instruction *VaddiI32(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs0, int32_t imm);
        Instruction *VaddiF32(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs0, float imm);    
        Instruction *VaddiF64(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs0, double imm);
        // sub (vlen)
        Instruction *VsubI32(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs0, uint32_t rs1);
        Instruction *VsubF32(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs0, uint32_t rs1);
        Instruction *VsubF64(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs0, uint32_t rs1);
        Instruction *VsubC32(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs0, uint32_t rs1);
        Instruction *VsubC64(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs0, uint32_t rs1);
        Instruction *VsubiI32(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs0, int32_t imm);
        Instruction *VsubiF32(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs0, float imm);    
        Instruction *VsubiF64(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs0, double imm);
        // mul (vlen)
        Instruction *VmulI32(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs0, uint32_t rs1);
        Instruction *VmulF32(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs0, uint32_t rs1);
        Instruction *VmulF64(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs0, uint32_t rs1);
        Instruction *VmulC32(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs0, uint32_t rs1);
        Instruction *VmuliI32(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs0, int32_t imm);
        Instruction *VmuliF32(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs0, float imm);    
        Instruction *VmuliF64(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs0, double imm);
        Instruction *VmuliC32(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs0, float _Complex imm);
        Instruction *VmuliC64(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs0, double _Complex imm);
        // gemm (xs ys zs)
        Instruction *GemmI32(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs0, uint32_t rs1);
        Instruction *GemmF32(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs0, uint32_t rs1);
        Instruction *GemmF64(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs0, uint32_t rs1);
        Instruction *GemmC32(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs0, uint32_t rs1);
        Instruction *GemmC64(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs0, uint32_t rs1);        
        // abs (vlen)
        Instruction *VabsI32(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs);
        Instruction *VabsF32(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs);
        Instruction *VabsF64(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs); 
        Instruction *VabsC32(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs);
        Instruction *VabsC64(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs);        
        // square (vlen)
        Instruction *VsquaI32(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs);
        Instruction *VsquaF32(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs);
        Instruction *VsquaF64(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs);
        // negative (vlen)
        Instruction *VnegI32(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs);
        Instruction *VnegF32(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs);
        Instruction *VnegF64(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs);
        // reciprocal (vlen)
        Instruction *VrecI32(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs);
        Instruction *VrecF32(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs);
        Instruction *VrecF64(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs);        
        // exp (vlen)
        Instruction *VexpI32(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs);
        Instruction *VexpF32(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs);
        Instruction *VexpF64(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs);
        // log10 (vlen)
        Instruction *Vlog10I32(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs);
        Instruction *Vlog10F32(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs);
        Instruction *Vlog10F64(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs);
        // conj (vlen)
        Instruction *VconjC32(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs);
        Instruction *VconjC64(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs);

        // vector aggregation operations
        // sum (vlen)
        Instruction *VsumI32(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs);
        Instruction *VsumF32(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs);
        Instruction *VsumF64(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs);
        // max (vlen)
        Instruction *VmaxI32(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs);
        Instruction *VmaxF32(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs);
        Instruction *VmaxF64(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs);       
        // min (vlen)
        Instruction *VminI32(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs);
        Instruction *VminF32(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs);
        Instruction *VminF64(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs);

        // transpose (ndim, xsize, ysize, zsize)
        Instruction *TransposeI32(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs);
        Instruction *TransposeF32(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs);
        Instruction *TransposeF64(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs);
        // permute (ndim, xsize, ysize, zsize, xaxis, yaxis, zaxis)
        Instruction *PermuteI32(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs);
        Instruction *PermuteF32(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs);
        Instruction *PermuteF64(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs);

        Instruction* Halt();
        Instruction* Call(const std::string& target, const std::string& dev, int path, int s, int n);
        Instruction* Ret();
        Instruction* Fence(uint32_t path);
        Instruction* Vload(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs0, uint32_t len);
        Instruction* Mload(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs0, uint32_t len);
        Instruction* Tload(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs0, uint32_t len);
        Instruction* Vstore(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs0, uint32_t len);
        Instruction* Mstore(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs0, uint32_t len);
        Instruction* Tstore(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs0, uint32_t len);
        Instruction* Mmp(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs0, uint32_t rs1);
        Instruction* Mma(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs0, uint32_t rs1);
        Instruction* Smm(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs0, uint32_t rs1);
        Instruction* Mclip(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs0, uint32_t rs1);
        Instruction* Mvp(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs0, uint32_t rs1);

        Instruction* MemSet(uint32_t dst, uint32_t len, uint32_t val);
        Instruction* Gemm(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs0, uint32_t rs1);

        // length of u: ULEN, length of v: VLEN
        Instruction* Conv(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs0, uint32_t rs1);
        Instruction* Fir(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs0, uint32_t rs1);
        // length of vector: VLEN
        Instruction* Fft(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs);
        Instruction* Ifft(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs);
        Instruction* Ddc(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs);
        Instruction* Extr(int path, Drive dri, Drive dro, uint32_t rd, uint32_t rs);

        Instruction* Display(const std::string& msg, uint32_t rs0);

        void Build();
        bool Valid();
        int PathNum();
        int GetEntry();
        int GetPC(const std::string& l);

        uint32_t Size();
        Instruction* operator[](uint32_t index);

    private:
        bool built;
        int path_num_;
        std::vector<std::string>   error_msgs_;
        std::vector<Instruction*>  insts_;
        std::map<std::string, int> labels_;
    };

    using ProgramPtr = std::shared_ptr<Program>;

}  // namespace tai

#endif //TAI_SIM_TAI_INST_H
