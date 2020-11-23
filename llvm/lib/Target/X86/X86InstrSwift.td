//===-- X86InstrInfo.td - Main X86 Instruction Definition --*- tablegen -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

//===----------------------------------------------------------------------===//
//  Swift Move Instructions.
//
let SchedRW = [WriteMove], isCodeGenOnly = 1 in {
def SWIFTMOV8rr  : I<0, Pseudo, (outs GR8 :$dst), (ins GR8 :$src),
                     "swiftmov{b}\t{$src, $dst|$dst, $src}",
                     [(set GR8:$dst, (int_x86_movswift GR8:$src))]>;
def SWIFTMOV16rr : I<0, Pseudo, (outs GR16:$dst), (ins GR16:$src),
                     "swiftmov{w}\t{$src, $dst|$dst, $src}",
                     [(set GR16:$dst, (int_x86_movswift GR16:$src))]
                     >, OpSize16;
def SWIFTMOV32rr : I<0, Pseudo, (outs GR32:$dst), (ins GR32:$src),
                     "swiftmov{l}\t{$src, $dst|$dst, $src}",
                     [(set GR32:$dst, (int_x86_movswift GR32:$src))]
                     >, OpSize32;
def SWIFTMOV64rr : RI<0, Pseudo, (outs GR64:$dst), (ins GR64:$src),
                      "swiftmov{q}\t{$src, $dst|$dst, $src}",
                      [(set GR64:$dst, (int_x86_movswift GR64:$src))]
                      >;
}

let isCodeGenOnly = 1 in {
def SWIFTMOVSSrr : SI<0, Pseudo, (outs FR32 :$dst), (ins FR32 :$src),
                     "fswiftmov{ss}\t{$src, $dst|$dst, $src}",
                     [(set FR32:$dst, (int_x86_fmovswift FR32:$src))]>, Sched<[WriteFShuffle]>;
def SWIFTMOVSDrr : SI<0, Pseudo, (outs FR64 :$dst), (ins FR64 :$src),
                     "fswiftmov{sd}\t{$src, $dst|$dst, $src}",
                     [(set FR64:$dst, (int_x86_fmovswift FR64:$src))]>, Sched<[WriteFShuffle]>;

// double quadword integers
def SWIFTMOVDQrr : PDI<0, Pseudo, (outs VR128 :$dst), (ins VR128 :$src),
                        "dqswiftmov\t{$src, $dst|$dst, $src}",
                        [(set VR128:$dst, (int_x86_dqmovswift VR128:$src))]>, Sched<[WriteMove]>;
// packed doubles
def SWIFTMOVPDrr : PDI<0, Pseudo, (outs VR128 :$dst), (ins VR128 :$src),
                        "pdswiftmov\t{$src, $dst|$dst, $src}",
                        [(set VR128:$dst, (int_x86_pdmovswift VR128:$src))]>, Sched<[WriteFShuffle]>;
// packed singles (floats)
def SWIFTMOVPSrr : PSI<0, Pseudo, (outs VR128 :$dst), (ins VR128 :$src),
                        "psswiftmov\t{$src, $dst|$dst, $src}",
                        [(set VR128:$dst, (int_x86_psmovswift VR128:$src))]>, Sched<[WriteFShuffle]>;
}

//===----------------------------------------------------------------------===//
//  Swift Compare Instructions.
//
// - We explicitly state that SwiftSub is not a comparison instruction
//   to avoid possible optimizations based on isCompare
let isCompare = 0, Constraints = "$src1 = $dst", Defs = [EFLAGS], SchedRW = [WriteALU], isCodeGenOnly = 1 in {

def SWIFTSUB8rr  : I<0, Pseudo, (outs GR8:$dst), (ins GR8:$src1, GR8:$src2),
                     "swiftsub{b}\t{$src2, $src1|$src1, $src2}",
                     [(set GR8:$dst, (int_x86_subswift GR8:$src1, GR8:$src2))]>;

def SWIFTSUB16rr : I<0, Pseudo, (outs GR16:$dst), (ins GR16:$src1, GR16:$src2),
                     "swiftsub{w}\t{$src2, $src1|$src1, $src2}",
                     [(set GR16:$dst, (int_x86_subswift GR16:$src1, GR16:$src2))]>, OpSize16;

def SWIFTSUB32rr : I<0, Pseudo, (outs GR32:$dst), (ins GR32:$src1, GR32:$src2),
                     "swiftsub{l}\t{$src2, $src1|$src1, $src2}",
                     [(set GR32:$dst, (int_x86_subswift GR32:$src1, GR32:$src2))]>, OpSize32;

def SWIFTSUB64rr : RI<0, Pseudo, (outs GR64:$dst), (ins GR64:$src1, GR64:$src2),
                      "swiftsub{q}\t{$src2, $src1|$src1, $src2}",
                      [(set GR64:$dst, (int_x86_subswift GR64:$src1, GR64:$src2))]>;

}

let isCompare = 0, Constraints = "$src1 = $dst", isCodeGenOnly = 1 in {

def SWIFTSUBSSrr : SI<0, Pseudo, (outs FR32:$dst), (ins FR32:$src1, FR32:$src2),
                      "fswiftsub{ss}\t{$src2, $dst|$dst, $src2}",
                      [(set FR32:$dst, (int_x86_fsubswift FR32:$src1, FR32:$src2))]>, Sched<[WriteFAdd]>;
def SWIFTSUBSDrr : SI<0, Pseudo, (outs FR64:$dst), (ins FR64:$src1, FR64:$src2),
                      "fswiftsub{sd}\t{$src2, $dst|$dst, $src2}",
                      [(set FR64:$dst, (int_x86_fsubswift FR64:$src1, FR64:$src2))]>, Sched<[WriteFAdd]>;

// double quadword integers
def SWIFTSUBDQrr : PDI<0, Pseudo, (outs VR128:$dst), (ins VR128:$src1, VR128:$src2),
                        "dqswiftsub\t{$src2, $dst|$dst, $src2}",
                        [(set VR128:$dst, (int_x86_dqsubswift VR128:$src1, VR128:$src2))]>, Sched<[WriteVecALU]>;
// packed doubles
def SWIFTSUBPDrr : PDI<0, Pseudo, (outs VR128:$dst), (ins VR128:$src1, VR128:$src2),
                        "pdswiftsub\t{$src2, $dst|$dst, $src2}",
                        [(set VR128:$dst, (int_x86_pdsubswift VR128:$src1, VR128:$src2))]>, Sched<[WriteFAdd]>;
// packed singles (floats)
def SWIFTSUBPSrr : PSI<0, Pseudo, (outs VR128:$dst), (ins VR128:$src1, VR128:$src2),
                        "psswiftsub\t{$src2, $dst|$dst, $src2}",
                        [(set VR128:$dst, (int_x86_pssubswift VR128:$src1, VR128:$src2))]>, Sched<[WriteFAdd]>;
}


//===----------------------------------------------------------------------===//
//  Swift Pseudo-Compare Instructions.
//  We use it to test NOP substitutions for compares. It does not define EFLAGS.
//
// - We explicitly state that SwiftSub is not a comparison instruction
//   to avoid possible optimizations based on isCompare
let isCompare = 0, Constraints = "$src1 = $dst", SchedRW = [WriteALU], isCodeGenOnly = 1 in {

def SWIFTNOPSUB8rr  : I<0, Pseudo, (outs GR8:$dst), (ins GR8:$src1, GR8:$src2),
                     "swiftnopsub{b}\t{$src2, $src1|$src1, $src2}",
                     [(set GR8:$dst, (int_x86_nopsubswift GR8:$src1, GR8:$src2))]>;

def SWIFTNOPSUB16rr : I<0, Pseudo, (outs GR16:$dst), (ins GR16:$src1, GR16:$src2),
                     "swiftnopsub{w}\t{$src2, $src1|$src1, $src2}",
                     [(set GR16:$dst, (int_x86_nopsubswift GR16:$src1, GR16:$src2))]>, OpSize16;

def SWIFTNOPSUB32rr : I<0, Pseudo, (outs GR32:$dst), (ins GR32:$src1, GR32:$src2),
                     "swiftnopsub{l}\t{$src2, $src1|$src1, $src2}",
                     [(set GR32:$dst, (int_x86_nopsubswift GR32:$src1, GR32:$src2))]>, OpSize32;

def SWIFTNOPSUB64rr : RI<0, Pseudo, (outs GR64:$dst), (ins GR64:$src1, GR64:$src2),
                      "swiftnopsub{q}\t{$src2, $src1|$src1, $src2}",
                      [(set GR64:$dst, (int_x86_nopsubswift GR64:$src1, GR64:$src2))]>;

}

let isCompare = 0, Constraints = "$src1 = $dst", isCodeGenOnly = 1 in {

def SWIFTNOPSUBSSrr : SI<0, Pseudo, (outs FR32:$dst), (ins FR32:$src1, FR32:$src2),
                      "fswiftnopsub{ss}\t{$src2, $dst|$dst, $src2}",
                      [(set FR32:$dst, (int_x86_fnopsubswift FR32:$src1, FR32:$src2))]>, Sched<[WriteFAdd]>;

def SWIFTNOPSUBSDrr : SI<0, Pseudo, (outs FR64:$dst), (ins FR64:$src1, FR64:$src2),
                      "fswiftnopsub{sd}\t{$src2, $dst|$dst, $src2}",
                      [(set FR64:$dst, (int_x86_fnopsubswift FR64:$src1, FR64:$src2))]>, Sched<[WriteFAdd]>;

// double quadword integers
def SWIFTNOPSUBDQrr : PDI<0, Pseudo, (outs VR128:$dst), (ins VR128:$src1, VR128:$src2),
                        "dqswiftnopsub\t{$src2, $dst|$dst, $src2}",
                        [(set VR128:$dst, (int_x86_dqnopsubswift VR128:$src1, VR128:$src2))]>, Sched<[WriteVecALU]>;
// packed doubles
def SWIFTNOPSUBPDrr : PDI<0, Pseudo, (outs VR128:$dst), (ins VR128:$src1, VR128:$src2),
                        "pdswiftnopsub\t{$src2, $dst|$dst, $src2}",
                        [(set VR128:$dst, (int_x86_pdnopsubswift VR128:$src1, VR128:$src2))]>, Sched<[WriteFAdd]>;
// packed singles (floats)
def SWIFTNOPSUBPSrr : PSI<0, Pseudo, (outs VR128:$dst), (ins VR128:$src1, VR128:$src2),
                        "psswiftnopsub\t{$src2, $dst|$dst, $src2}",
                        [(set VR128:$dst, (int_x86_psnopsubswift VR128:$src1, VR128:$src2))]>, Sched<[WriteFAdd]>;

}

class SwiftBinOpRR_F<string mnemonic, X86TypeInfo typeinfo>
  : BinOpRR<0, mnemonic, typeinfo, (outs), WriteALU, //MRMDestReg,
            [(set EFLAGS,
                  (X86swiftcmp typeinfo.RegClass:$src1, typeinfo.RegClass:$src2))] >;

let Defs = [EFLAGS], isCodeGenOnly = 1 in {
  def SWIFTCMP8rr  : SwiftBinOpRR_F<"swiftcmp{b}", Xi8>;
  def SWIFTCMP16rr : SwiftBinOpRR_F<"swiftcmp{w}", Xi16>;
  def SWIFTCMP32rr : SwiftBinOpRR_F<"swiftcmp{l}", Xi32>;
  def SWIFTCMP64rr : SwiftBinOpRR_F<"swiftcmp{q}", Xi64>;
}
