//===-- X86FixupLEAs.cpp - use or replace LEA instructions -----------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file defines the pass that finds Swift pseudo-instructions and
// substitutes them by the real instructions.
//
//===----------------------------------------------------------------------===//

#include "X86.h"
#include "X86InstrInfo.h"
#include "X86Subtarget.h"
#include "llvm/CodeGen/LiveVariables.h"
#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/Passes.h"
#include "llvm/Support/Debug.h"
#include "llvm/Support/raw_ostream.h"
#include "llvm/CodeGen/TargetInstrInfo.h"
using namespace llvm;

#define DEBUG_TYPE "x86-replace-swift"


namespace {
class ReplaceSwiftPass : public MachineFunctionPass {
  static char ID;

  StringRef getPassName() const override { return "X86 Replacement of SWIFT pseudo-instructions"; }

public:
  ReplaceSwiftPass();

  bool processInstruction(MachineBasicBlock::iterator &MI, MachineFunction::iterator &MBB,
                          MachineFunction &MF);

  bool runOnMachineFunction(MachineFunction &MF) override;

private:
  const TargetMachine *TM;
  const X86InstrInfo *TII;
  std::map<unsigned, unsigned> OpSwiftToX86;
  unsigned regSwiftSub;
};

char ReplaceSwiftPass::ID = 0;
}

ReplaceSwiftPass::ReplaceSwiftPass()
  : MachineFunctionPass(ID) {

  // moves
  OpSwiftToX86[X86::SWIFTMOV8rr]  = X86::MOV8rr;
  OpSwiftToX86[X86::SWIFTMOV16rr] = X86::MOV16rr;
  OpSwiftToX86[X86::SWIFTMOV32rr] = X86::MOV32rr;
  OpSwiftToX86[X86::SWIFTMOV64rr] = X86::MOV64rr;

  OpSwiftToX86[X86::SWIFTMOVSSrr] = X86::MOVSSrr;
  OpSwiftToX86[X86::SWIFTMOVSDrr] = X86::MOVSDrr;

  OpSwiftToX86[X86::SWIFTMOVDQrr] = X86::MOVDQArr;
  OpSwiftToX86[X86::SWIFTMOVPDrr] = X86::MOVAPDrr;
  OpSwiftToX86[X86::SWIFTMOVPSrr] = X86::MOVAPSrr;

  // subs
  OpSwiftToX86[X86::SWIFTSUB8rr]  = X86::SUB8rr;
  OpSwiftToX86[X86::SWIFTSUB16rr] = X86::SUB16rr;
  OpSwiftToX86[X86::SWIFTSUB32rr] = X86::SUB32rr;
  OpSwiftToX86[X86::SWIFTSUB64rr] = X86::SUB64rr;

  OpSwiftToX86[X86::SWIFTSUBSSrr] = X86::SUBSSrr;
  OpSwiftToX86[X86::SWIFTSUBSDrr] = X86::SUBSDrr;

  OpSwiftToX86[X86::SWIFTSUBDQrr] = X86::PSUBBrr;
  OpSwiftToX86[X86::SWIFTSUBPDrr] = X86::SUBPDrr;
  OpSwiftToX86[X86::SWIFTSUBPSrr] = X86::SUBPSrr;

  // cmps
  OpSwiftToX86[X86::SWIFTCMP8rr]  = X86::CMP8rr;
  OpSwiftToX86[X86::SWIFTCMP16rr] = X86::CMP16rr;
  OpSwiftToX86[X86::SWIFTCMP32rr] = X86::CMP32rr;
  OpSwiftToX86[X86::SWIFTCMP64rr] = X86::CMP64rr;

  /* TODO: add SWIFTCMP for SSE! */
}

bool ReplaceSwiftPass::processInstruction(MachineBasicBlock::iterator &MI,
                                          MachineFunction::iterator &MBB,
                                          MachineFunction &MF) {
/*
  if (regSwiftSub != X86::NoRegister) {
 if (MI->isCompare() &&
   MI->getNumOperands() > 0 &&
   MI->getOperand(0).isReg() &&
   MI->getOperand(0).getReg() == regSwiftSub) {
   // found redundant compare instr (`cmp reg, 0x0`) after swiftsub
     MachineInstr *NewMI = BuildMI(MF, MI->getDebugLoc(), TII->get(X86::NOOPL))
                 .addReg(X86::RAX)
                 .addImm(1)
                 .addReg(0)
                 .addImm(0)
                 .addReg(0);
     MBB->insert(MI, NewMI);

     MBB->erase(MI);
     regSwiftSub = X86::NoRegister;
 }
  }
*/
  int Opcode = MI->getOpcode();
  switch (Opcode) {
  case X86::SWIFTMOV8rr:
  case X86::SWIFTMOV16rr:
  case X86::SWIFTMOV32rr:
  case X86::SWIFTMOV64rr:
  case X86::SWIFTCMP8rr:
  case X86::SWIFTCMP16rr:
  case X86::SWIFTCMP32rr:
  case X86::SWIFTCMP64rr:
  case X86::SWIFTMOVDQrr:
  case X86::SWIFTMOVPDrr:
  case X86::SWIFTMOVPSrr: {
//    DEBUG(dbgs() << "Replacing: "; MI->dump());
    const MachineOperand &Dest = MI->getOperand(0);
    const MachineOperand &Src = MI->getOperand(1);

    unsigned NewOpcode = OpSwiftToX86[Opcode];
    MachineInstr *NewMI = BuildMI(MF, MI->getDebugLoc(), TII->get(NewOpcode))
            .add(Dest)
            .add(Src);
    MBB->insert(MI, NewMI);
    MBB->erase(MI);
//    DEBUG(dbgs() << "New instruction: "; NewMI->dump());
    return true;
  }

  case X86::SWIFTMOVSSrr:
  case X86::SWIFTMOVSDrr: {
//    DEBUG(dbgs() << "Replacing: "; MI->dump());
    const MachineOperand &Dest = MI->getOperand(0);
    const MachineOperand &Src = MI->getOperand(1);

    unsigned NewOpcode = OpSwiftToX86[Opcode];
    MachineInstr *NewMI = BuildMI(MF, MI->getDebugLoc(), TII->get(NewOpcode))
      .add(Dest)
      .add(Dest)
      .add(Src);
    MBB->insert(MI, NewMI);
    MBB->erase(MI);
//    DEBUG(dbgs() << "New instruction: "; NewMI->dump());
    return true;
  }

  case X86::SWIFTSUB8rr:
  case X86::SWIFTSUB16rr:
  case X86::SWIFTSUB32rr:
  case X86::SWIFTSUB64rr: {
//    DEBUG(dbgs() << "Replacing: "; MI->dump());
    const MachineOperand &Dest = MI->getOperand(0);
    const MachineOperand &Src1 = MI->getOperand(1);
    const MachineOperand &Src2 = MI->getOperand(2);
    const MachineOperand &Flag = MI->getOperand(3);

    unsigned NewOpcode = OpSwiftToX86[Opcode];
    MachineInstr *NewMI = BuildMI(MF, MI->getDebugLoc(), TII->get(NewOpcode))
                .add(Dest)
                .add(Src1)
                .add(Src2)
                .add(Flag);
    MBB->insert(MI, NewMI);

    assert(Dest.isReg() && "Dest operand in SWIFTSUB is not a register!");
    regSwiftSub = Dest.getReg();
    MBB->erase(MI);

//    DEBUG(dbgs() << "New instruction: "; NewMI->dump());
    return true;
  }

  case X86::SWIFTSUBSSrr:
  case X86::SWIFTSUBSDrr:
  case X86::SWIFTSUBDQrr:
  case X86::SWIFTSUBPDrr:
  case X86::SWIFTSUBPSrr: {
//    DEBUG(dbgs() << "Replacing: "; MI->dump());
    const MachineOperand &Dest = MI->getOperand(0);
    const MachineOperand &Src1 = MI->getOperand(1);
    const MachineOperand &Src2 = MI->getOperand(2);

    unsigned NewOpcode = OpSwiftToX86[Opcode];
    MachineInstr *NewMI = BuildMI(MF, MI->getDebugLoc(), TII->get(NewOpcode))
                .add(Dest)
                .add(Src1)
                .add(Src2);
    MBB->insert(MI, NewMI);
    MBB->erase(MI);
//    DEBUG(dbgs() << "New instruction: "; NewMI->dump());
    return true;
  }

  case X86::SWIFTNOPSUB8rr:
  case X86::SWIFTNOPSUB16rr:
  case X86::SWIFTNOPSUB32rr:
  case X86::SWIFTNOPSUB64rr: {
//    DEBUG(dbgs() << "Replacing: "; MI->dump());

    // taken from lib/Target/X86/X86MCInstLower.cpp
    // 3 byte NOOP is `nopl (%rax)`
    unsigned NewOpcode, BaseReg, ScaleVal, IndexReg, Displacement, SegmentReg;
    IndexReg = Displacement = SegmentReg = 0;
    NewOpcode = X86::NOOPL;
    BaseReg = X86::RAX;
    ScaleVal = 1;

    MachineInstr *NewMI = BuildMI(MF, MI->getDebugLoc(), TII->get(NewOpcode))
                .addReg(BaseReg)
                .addImm(ScaleVal)
                .addReg(IndexReg)
                .addImm(Displacement)
                .addReg(SegmentReg);
    MBB->insert(MI, NewMI);
//    DEBUG(dbgs() << "New instruction: "; NewMI->dump());
    MBB->erase(MI);
    return true;
  }

  case X86::SWIFTNOPSUBSSrr:
  case X86::SWIFTNOPSUBSDrr:
  case X86::SWIFTNOPSUBDQrr:
  case X86::SWIFTNOPSUBPDrr:
  case X86::SWIFTNOPSUBPSrr: {
//    DEBUG(dbgs() << "Replacing: "; MI->dump());

    // 4 byte NOOP is `nopl 0x0(%rax)`
    unsigned NewOpcode, BaseReg, ScaleVal, IndexReg, Displacement, SegmentReg;
    IndexReg = SegmentReg = 0;
    NewOpcode = X86::NOOPL;
    BaseReg = X86::RAX;
    ScaleVal = 1;
    Displacement = 8;

    MachineInstr *NewMI = BuildMI(MF, MI->getDebugLoc(), TII->get(NewOpcode))
                .addReg(BaseReg)
                .addImm(ScaleVal)
                .addReg(IndexReg)
                .addImm(Displacement)
                .addReg(SegmentReg);
    MBB->insert(MI, NewMI);
//    DEBUG(dbgs() << "New instruction: "; NewMI->dump());
    MBB->erase(MI);
    return true;
  }
  }

  return false;
}


bool ReplaceSwiftPass::runOnMachineFunction(MachineFunction &MF) {
  TM = &MF.getTarget();
  TII = static_cast<const X86InstrInfo *>(TM->getMCInstrInfo());

  bool modified = false;
  regSwiftSub = X86::NoRegister;

//  DEBUG(dbgs() << "Start X86ReplaceSwift\n";);
  for (MachineFunction::iterator BBI = MF.begin(), BBE = MF.end(); BBI != BBE; ++BBI) {
    MachineBasicBlock::iterator I = BBI->begin(), E = BBI->end();
    while (I != E) {
      MachineBasicBlock::iterator N = std::next(I);
      modified |= processInstruction(I, BBI, MF);
      I = N;
    }
  }
//  DEBUG(dbgs() << "End X86ReplaceSwift\n";);

  return modified;
}

FunctionPass *llvm::createX86ReplaceSwift() {
  return new ReplaceSwiftPass();
}
