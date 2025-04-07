//-----------------------------------------------------------------
//                         RISC-V Core
//                            V1.0.1
//                     Ultra-Embedded.com
//                     Copyright 2014-2019
//
//                   admin@ultra-embedded.com
//
//                       License: BSD
//-----------------------------------------------------------------
//
// Copyright (c) 2014-2019, Ultra-Embedded.com
// All rights reserved.
// 
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions 
// are met:
//   - Redistributions of source code must retain the above copyright
//     notice, this list of conditions and the following disclaimer.
//   - Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer 
//     in the documentation and/or other materials provided with the 
//     distribution.
//   - Neither the name of the author nor the names of its contributors 
//     may be used to endorse or promote products derived from this 
//     software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR 
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE 
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR 
// BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF 
// THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF 
// SUCH DAMAGE.
//-----------------------------------------------------------------
module riscv_exec
(
    // Inputs
     input           clk_i
    ,input           rst_i
    ,input           opcode_valid_i
    ,input  [ 31:0]  opcode_opcode_i
    ,input  [ 31:0]  opcode_pc_i
    ,input           opcode_invalid_i
    ,input  [  4:0]  opcode_rd_idx_i
    ,input  [  4:0]  opcode_ra_idx_i
    ,input  [  4:0]  opcode_rb_idx_i
    ,input  [ 31:0]  opcode_ra_operand_i
    ,input  [ 31:0]  opcode_rb_operand_i
    ,input           hold_i
    // Outputs
    ,output          branch_request_o
    ,output          branch_is_taken_o
    ,output          branch_is_not_taken_o
    ,output [ 31:0]  branch_source_o
    ,output          branch_is_call_o
    ,output          branch_is_ret_o
    ,output          branch_is_jmp_o
    ,output [ 31:0]  branch_pc_o
    ,output          branch_d_request_o
    ,output [ 31:0]  branch_d_pc_o
    ,output [  1:0]  branch_d_priv_o
    ,output [ 31:0]  writeback_value_o
);
//-----------------------------------------------------------------
// Includes
//-----------------------------------------------------------------
`include "riscv_defs.v"//-----------------------------------------------------------------
//                         RISC-V Core
//                            V1.0.1
//                     Ultra-Embedded.com
//                     Copyright 2014-2019
//
//                   admin@ultra-embedded.com
//
//                       License: BSD
//-----------------------------------------------------------------
//
// Copyright (c) 2014-2019, Ultra-Embedded.com
// All rights reserved.
// 
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions 
// are met:
//   - Redistributions of source code must retain the above copyright
//     notice, this list of conditions and the following disclaimer.
//   - Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer 
//     in the documentation and/or other materials provided with the 
//     distribution.
//   - Neither the name of the author nor the names of its contributors 
//     may be used to endorse or promote products derived from this 
//     software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR 
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE 
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR 
// BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF 
// THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF 
// SUCH DAMAGE.
//-----------------------------------------------------------------
//--------------------------------------------------------------------
// ALU Operations
//--------------------------------------------------------------------
`define ALU_NONE                                4'b0000
`define ALU_SHIFTL                              4'b0001
`define ALU_SHIFTR                              4'b0010
`define ALU_SHIFTR_ARITH                        4'b0011
`define ALU_ADD                                 4'b0100
`define ALU_SUB                                 4'b0110
`define ALU_AND                                 4'b0111
`define ALU_OR                                  4'b1000
`define ALU_XOR                                 4'b1001
`define ALU_LESS_THAN                           4'b1010
`define ALU_LESS_THAN_SIGNED                    4'b1011
//--------------------------------------------------------------------
// Instructions Masks
//--------------------------------------------------------------------
// andi
`define INST_ANDI 32'h7013
`define INST_ANDI_MASK 32'h707f
// addi
`define INST_ADDI 32'h13
`define INST_ADDI_MASK 32'h707f
// slti
`define INST_SLTI 32'h2013
`define INST_SLTI_MASK 32'h707f
// sltiu
`define INST_SLTIU 32'h3013
`define INST_SLTIU_MASK 32'h707f
// ori
`define INST_ORI 32'h6013
`define INST_ORI_MASK 32'h707f
// xori
`define INST_XORI 32'h4013
`define INST_XORI_MASK 32'h707f
// slli
`define INST_SLLI 32'h1013
`define INST_SLLI_MASK 32'hfc00707f
// srli
`define INST_SRLI 32'h5013
`define INST_SRLI_MASK 32'hfc00707f
// srai
`define INST_SRAI 32'h40005013
`define INST_SRAI_MASK 32'hfc00707f
// lui
`define INST_LUI 32'h37
`define INST_LUI_MASK 32'h7f
// auipc
`define INST_AUIPC 32'h17
`define INST_AUIPC_MASK 32'h7f
// add
`define INST_ADD 32'h33
`define INST_ADD_MASK 32'hfe00707f
// sub
`define INST_SUB 32'h40000033
`define INST_SUB_MASK 32'hfe00707f
// slt
`define INST_SLT 32'h2033
`define INST_SLT_MASK 32'hfe00707f
// sltu
`define INST_SLTU 32'h3033
`define INST_SLTU_MASK 32'hfe00707f
// xor
`define INST_XOR 32'h4033
`define INST_XOR_MASK 32'hfe00707f
// or
`define INST_OR 32'h6033
`define INST_OR_MASK 32'hfe00707f
// and
`define INST_AND 32'h7033
`define INST_AND_MASK 32'hfe00707f
// sll
`define INST_SLL 32'h1033
`define INST_SLL_MASK 32'hfe00707f
// srl
`define INST_SRL 32'h5033
`define INST_SRL_MASK 32'hfe00707f
// sra
`define INST_SRA 32'h40005033
`define INST_SRA_MASK 32'hfe00707f
// jal
`define INST_JAL 32'h6f
`define INST_JAL_MASK 32'h7f
// jalr
`define INST_JALR 32'h67
`define INST_JALR_MASK 32'h707f
// beq
`define INST_BEQ 32'h63
`define INST_BEQ_MASK 32'h707f
// bne
`define INST_BNE 32'h1063
`define INST_BNE_MASK 32'h707f
// blt
`define INST_BLT 32'h4063
`define INST_BLT_MASK 32'h707f
// bge
`define INST_BGE 32'h5063
`define INST_BGE_MASK 32'h707f
// bltu
`define INST_BLTU 32'h6063
`define INST_BLTU_MASK 32'h707f
// bgeu
`define INST_BGEU 32'h7063
`define INST_BGEU_MASK 32'h707f
// lb
`define INST_LB 32'h3
`define INST_LB_MASK 32'h707f
// lh
`define INST_LH 32'h1003
`define INST_LH_MASK 32'h707f
// lw
`define INST_LW 32'h2003
`define INST_LW_MASK 32'h707f
// lbu
`define INST_LBU 32'h4003
`define INST_LBU_MASK 32'h707f
// lhu
`define INST_LHU 32'h5003
`define INST_LHU_MASK 32'h707f
// lwu
`define INST_LWU 32'h6003
`define INST_LWU_MASK 32'h707f
// sb
`define INST_SB 32'h23
`define INST_SB_MASK 32'h707f
// sh
`define INST_SH 32'h1023
`define INST_SH_MASK 32'h707f
// sw
`define INST_SW 32'h2023
`define INST_SW_MASK 32'h707f
// ecall
`define INST_ECALL 32'h73
`define INST_ECALL_MASK 32'hffffffff
// ebreak
`define INST_EBREAK 32'h100073
`define INST_EBREAK_MASK 32'hffffffff
// eret
`define INST_ERET 32'h200073
`define INST_ERET_MASK 32'hcfffffff
// csrrw
`define INST_CSRRW 32'h1073
`define INST_CSRRW_MASK 32'h707f
// csrrs
`define INST_CSRRS 32'h2073
`define INST_CSRRS_MASK 32'h707f
// csrrc
`define INST_CSRRC 32'h3073
`define INST_CSRRC_MASK 32'h707f
// csrrwi
`define INST_CSRRWI 32'h5073
`define INST_CSRRWI_MASK 32'h707f
// csrrsi
`define INST_CSRRSI 32'h6073
`define INST_CSRRSI_MASK 32'h707f
// csrrci
`define INST_CSRRCI 32'h7073
`define INST_CSRRCI_MASK 32'h707f
// mul
`define INST_MUL 32'h2000033
`define INST_MUL_MASK 32'hfe00707f
// mulh
`define INST_MULH 32'h2001033
`define INST_MULH_MASK 32'hfe00707f
// mulhsu
`define INST_MULHSU 32'h2002033
`define INST_MULHSU_MASK 32'hfe00707f
// mulhu
`define INST_MULHU 32'h2003033
`define INST_MULHU_MASK 32'hfe00707f
// div
`define INST_DIV 32'h2004033
`define INST_DIV_MASK 32'hfe00707f
// divu
`define INST_DIVU 32'h2005033
`define INST_DIVU_MASK 32'hfe00707f
// rem
`define INST_REM 32'h2006033
`define INST_REM_MASK 32'hfe00707f
// remu
`define INST_REMU 32'h2007033
`define INST_REMU_MASK 32'hfe00707f
// wfi
`define INST_WFI 32'h10500073
`define INST_WFI_MASK 32'hffff8fff
// fence
`define INST_FENCE 32'hf
`define INST_FENCE_MASK 32'h707f
// sfence
`define INST_SFENCE 32'h12000073
`define INST_SFENCE_MASK 32'hfe007fff
// fence.i
`define INST_IFENCE 32'h100f
`define INST_IFENCE_MASK 32'h707f
//--------------------------------------------------------------------
// Privilege levels
//--------------------------------------------------------------------
`define PRIV_USER         2'd0
`define PRIV_SUPER        2'd1
`define PRIV_MACHINE      2'd3
//--------------------------------------------------------------------
// IRQ Numbers
//--------------------------------------------------------------------
`define IRQ_S_SOFT   1
`define IRQ_M_SOFT   3
`define IRQ_S_TIMER  5
`define IRQ_M_TIMER  7
`define IRQ_S_EXT    9
`define IRQ_M_EXT    11
`define IRQ_MIN      (`IRQ_S_SOFT)
`define IRQ_MAX      (`IRQ_M_EXT + 1)
`define IRQ_MASK     ((1 << `IRQ_M_EXT)   | (1 << `IRQ_S_EXT)   |                       (1 << `IRQ_M_TIMER) | (1 << `IRQ_S_TIMER) |                       (1 << `IRQ_M_SOFT)  | (1 << `IRQ_S_SOFT))
`define SR_IP_MSIP_R      `IRQ_M_SOFT
`define SR_IP_MTIP_R      `IRQ_M_TIMER
`define SR_IP_MEIP_R      `IRQ_M_EXT
`define SR_IP_SSIP_R      `IRQ_S_SOFT
`define SR_IP_STIP_R      `IRQ_S_TIMER
`define SR_IP_SEIP_R      `IRQ_S_EXT
//--------------------------------------------------------------------
// CSR Registers - Simulation control
//--------------------------------------------------------------------
`define CSR_DSCRATCH       12'h7b2
`define CSR_SIM_CTRL       12'h8b2
`define CSR_SIM_CTRL_MASK  32'hFFFFFFFF
    `define CSR_SIM_CTRL_EXIT (0 << 24)
    `define CSR_SIM_CTRL_PUTC (1 << 24)
//--------------------------------------------------------------------
// CSR Registers
//--------------------------------------------------------------------
`define CSR_MSTATUS       12'h300
`define CSR_MSTATUS_MASK  32'hFFFFFFFF
`define CSR_MISA          12'h301
`define CSR_MISA_MASK     32'hFFFFFFFF
    `define MISA_RV32     32'h40000000
    `define MISA_RVI      32'h00000100
    `define MISA_RVE      32'h00000010
    `define MISA_RVM      32'h00001000
    `define MISA_RVA      32'h00000001
    `define MISA_RVF      32'h00000020
    `define MISA_RVD      32'h00000008
    `define MISA_RVC      32'h00000004
    `define MISA_RVS      32'h00040000
    `define MISA_RVU      32'h00100000
`define CSR_MEDELEG       12'h302
`define CSR_MEDELEG_MASK  32'h0000FFFF
`define CSR_MIDELEG       12'h303
`define CSR_MIDELEG_MASK  32'h0000FFFF
`define CSR_MIE           12'h304
`define CSR_MIE_MASK      `IRQ_MASK
`define CSR_MTVEC         12'h305
`define CSR_MTVEC_MASK    32'hFFFFFFFF
`define CSR_MSCRATCH      12'h340
`define CSR_MSCRATCH_MASK 32'hFFFFFFFF
`define CSR_MEPC          12'h341
`define CSR_MEPC_MASK     32'hFFFFFFFF
`define CSR_MCAUSE        12'h342
`define CSR_MCAUSE_MASK   32'h8000000F
`define CSR_MTVAL         12'h343
`define CSR_MTVAL_MASK    32'hFFFFFFFF
`define CSR_MIP           12'h344
`define CSR_MIP_MASK      `IRQ_MASK
`define CSR_MCYCLE        12'hc00
`define CSR_MCYCLE_MASK   32'hFFFFFFFF
`define CSR_MTIME         12'hc01
`define CSR_MTIME_MASK    32'hFFFFFFFF
`define CSR_MTIMEH        12'hc81
`define CSR_MTIMEH_MASK   32'hFFFFFFFF
`define CSR_MHARTID       12'hF14
`define CSR_MHARTID_MASK  32'hFFFFFFFF
// Non-std
`define CSR_MTIMECMP        12'h7c0
`define CSR_MTIMECMP_MASK   32'hFFFFFFFF
//-----------------------------------------------------------------
// CSR Registers - Supervisor
//-----------------------------------------------------------------
`define CSR_SSTATUS       12'h100
`define CSR_SSTATUS_MASK  `SR_SMODE_MASK
`define CSR_SIE           12'h104
`define CSR_SIE_MASK      ((1 << `IRQ_S_EXT) | (1 << `IRQ_S_TIMER) | (1 << `IRQ_S_SOFT))
`define CSR_STVEC         12'h105
`define CSR_STVEC_MASK    32'hFFFFFFFF
`define CSR_SSCRATCH      12'h140
`define CSR_SSCRATCH_MASK 32'hFFFFFFFF
`define CSR_SEPC          12'h141
`define CSR_SEPC_MASK     32'hFFFFFFFF
`define CSR_SCAUSE        12'h142
`define CSR_SCAUSE_MASK   32'h8000000F
`define CSR_STVAL         12'h143
`define CSR_STVAL_MASK    32'hFFFFFFFF
`define CSR_SIP           12'h144
`define CSR_SIP_MASK      ((1 << `IRQ_S_EXT) | (1 << `IRQ_S_TIMER) | (1 << `IRQ_S_SOFT))
`define CSR_SATP          12'h180
`define CSR_SATP_MASK     32'hFFFFFFFF
//--------------------------------------------------------------------
// CSR Registers - DCACHE control
//--------------------------------------------------------------------
`define CSR_DFLUSH            12'h3a0 // pmpcfg0
`define CSR_DFLUSH_MASK       32'hFFFFFFFF
`define CSR_DWRITEBACK        12'h3a1 // pmpcfg1
`define CSR_DWRITEBACK_MASK   32'hFFFFFFFF
`define CSR_DINVALIDATE       12'h3a2 // pmpcfg2
`define CSR_DINVALIDATE_MASK  32'hFFFFFFFF
//--------------------------------------------------------------------
// Status Register
//--------------------------------------------------------------------
`define SR_UIE         (1 << 0)
`define SR_UIE_R       0
`define SR_SIE         (1 << 1)
`define SR_SIE_R       1
`define SR_MIE         (1 << 3)
`define SR_MIE_R       3
`define SR_UPIE        (1 << 4)
`define SR_UPIE_R      4
`define SR_SPIE        (1 << 5)
`define SR_SPIE_R      5
`define SR_MPIE        (1 << 7)
`define SR_MPIE_R      7
`define SR_SPP         (1 << 8)
`define SR_SPP_R       8
`define SR_MPP_SHIFT   11
`define SR_MPP_MASK    2'h3
`define SR_MPP_R       12:11
`define SR_MPP_U       `PRIV_USER
`define SR_MPP_S       `PRIV_SUPER
`define SR_MPP_M       `PRIV_MACHINE
`define SR_SUM_R        18
`define SR_SUM          (1 << `SR_SUM_R)
`define SR_MPRV_R       17
`define SR_MPRV         (1 << `SR_MPRV_R)
`define SR_MXR_R        19
`define SR_MXR          (1 << `SR_MXR_R)
`define SR_SMODE_MASK   (`SR_UIE | `SR_SIE | `SR_UPIE | `SR_SPIE | `SR_SPP | `SR_SUM)
//--------------------------------------------------------------------
// SATP definitions
//--------------------------------------------------------------------
`define SATP_PPN_R        19:0 // TODO: Should be 21??
`define SATP_ASID_R       30:22
`define SATP_MODE_R       31
//--------------------------------------------------------------------
// MMU Defs (SV32)
//--------------------------------------------------------------------
`define MMU_LEVELS        2
`define MMU_PTIDXBITS     10
`define MMU_PTESIZE       4
`define MMU_PGSHIFT       (`MMU_PTIDXBITS + 2)
`define MMU_PGSIZE        (1 << `MMU_PGSHIFT)
`define MMU_VPN_BITS      (`MMU_PTIDXBITS * `MMU_LEVELS)
`define MMU_PPN_BITS      (32 - `MMU_PGSHIFT)
`define MMU_VA_BITS       (`MMU_VPN_BITS + `MMU_PGSHIFT)
`define PAGE_PRESENT      0
`define PAGE_READ         1
`define PAGE_WRITE        2
`define PAGE_EXEC         3
`define PAGE_USER         4
`define PAGE_GLOBAL       5
`define PAGE_ACCESSED     6
`define PAGE_DIRTY        7
`define PAGE_SOFT         9:8
`define PAGE_FLAGS       10'h3FF
`define PAGE_PFN_SHIFT   10
`define PAGE_SIZE        4096
//--------------------------------------------------------------------
// Exception Causes
//--------------------------------------------------------------------
`define EXCEPTION_W                        6
`define EXCEPTION_MISALIGNED_FETCH         6'h10
`define EXCEPTION_FAULT_FETCH              6'h11
`define EXCEPTION_ILLEGAL_INSTRUCTION      6'h12
`define EXCEPTION_BREAKPOINT               6'h13
`define EXCEPTION_MISALIGNED_LOAD          6'h14
`define EXCEPTION_FAULT_LOAD               6'h15
`define EXCEPTION_MISALIGNED_STORE         6'h16
`define EXCEPTION_FAULT_STORE              6'h17
`define EXCEPTION_ECALL                    6'h18
`define EXCEPTION_ECALL_U                  6'h18
`define EXCEPTION_ECALL_S                  6'h19
`define EXCEPTION_ECALL_H                  6'h1a
`define EXCEPTION_ECALL_M                  6'h1b
`define EXCEPTION_PAGE_FAULT_INST          6'h1c
`define EXCEPTION_PAGE_FAULT_LOAD          6'h1d
`define EXCEPTION_PAGE_FAULT_STORE         6'h1f
`define EXCEPTION_EXCEPTION                6'h10
`define EXCEPTION_INTERRUPT                6'h20
`define EXCEPTION_ERET_U                   6'h30
`define EXCEPTION_ERET_S                   6'h31
`define EXCEPTION_ERET_H                   6'h32
`define EXCEPTION_ERET_M                   6'h33
`define EXCEPTION_FENCE                    6'h34
`define EXCEPTION_TYPE_MASK                6'h30
`define EXCEPTION_SUBTYPE_R                3:0
`define MCAUSE_INT                      31
`define MCAUSE_MISALIGNED_FETCH         ((0 << `MCAUSE_INT) | 0)
`define MCAUSE_FAULT_FETCH              ((0 << `MCAUSE_INT) | 1)
`define MCAUSE_ILLEGAL_INSTRUCTION      ((0 << `MCAUSE_INT) | 2)
`define MCAUSE_BREAKPOINT               ((0 << `MCAUSE_INT) | 3)
`define MCAUSE_MISALIGNED_LOAD          ((0 << `MCAUSE_INT) | 4)
`define MCAUSE_FAULT_LOAD               ((0 << `MCAUSE_INT) | 5)
`define MCAUSE_MISALIGNED_STORE         ((0 << `MCAUSE_INT) | 6)
`define MCAUSE_FAULT_STORE              ((0 << `MCAUSE_INT) | 7)
`define MCAUSE_ECALL_U                  ((0 << `MCAUSE_INT) | 8)
`define MCAUSE_ECALL_S                  ((0 << `MCAUSE_INT) | 9)
`define MCAUSE_ECALL_H                  ((0 << `MCAUSE_INT) | 10)
`define MCAUSE_ECALL_M                  ((0 << `MCAUSE_INT) | 11)
`define MCAUSE_PAGE_FAULT_INST          ((0 << `MCAUSE_INT) | 12)
`define MCAUSE_PAGE_FAULT_LOAD          ((0 << `MCAUSE_INT) | 13)
`define MCAUSE_PAGE_FAULT_STORE         ((0 << `MCAUSE_INT) | 15)
`define MCAUSE_INTERRUPT                (1 << `MCAUSE_INT)
//--------------------------------------------------------------------
// Debug
//--------------------------------------------------------------------
`define RISCV_REGNO_FIRST   13'd0
`define RISCV_REGNO_GPR0    13'd0
`define RISCV_REGNO_GPR31   13'd31
`define RISCV_REGNO_PC      13'd32
`define RISCV_REGNO_CSR0    13'd65
`define RISCV_REGNO_CSR4095 (`RISCV_REGNO_CSR0 +  13'd4095)
`define RISCV_REGNO_PRIV    13'd4161
//-------------------------------------------------------------
// Opcode decode
//-------------------------------------------------------------
reg [31:0]  imm20_r;
reg [31:0]  imm12_r;
reg [31:0]  bimm_r;
reg [31:0]  jimm20_r;
reg [4:0]   shamt_r;
always @ *
begin
    imm20_r     = {opcode_opcode_i[31:12], 12'b0};
    imm12_r     = {{20{opcode_opcode_i[31]}}, opcode_opcode_i[31:20]};
    bimm_r      = {{19{opcode_opcode_i[31]}}, opcode_opcode_i[31], opcode_opcode_i[7], opcode_opcode_i[30:25], opcode_opcode_i[11:8], 1'b0};
    jimm20_r    = {{12{opcode_opcode_i[31]}}, opcode_opcode_i[19:12], opcode_opcode_i[20], opcode_opcode_i[30:25], opcode_opcode_i[24:21], 1'b0};
    shamt_r     = opcode_opcode_i[24:20];
end
//-------------------------------------------------------------
// Execute - ALU operations
//-------------------------------------------------------------
reg [3:0]  alu_func_r;
reg [31:0] alu_input_a_r;
reg [31:0] alu_input_b_r;
always @ *
begin
    alu_func_r     = `ALU_NONE4'b0000;
    alu_input_a_r  = 32'b0;
    alu_input_b_r  = 32'b0;
    if ((opcode_opcode_i & `INST_ADD_MASK32'hfe00707f) == `INST_ADD32'h33) // add
    begin
        alu_func_r     = `ALU_ADD4'b0100;
        alu_input_a_r  = opcode_ra_operand_i;
        alu_input_b_r  = opcode_rb_operand_i;
    end
    else if ((opcode_opcode_i & `INST_AND_MASK32'hfe00707f) == `INST_AND32'h7033) // and
    begin
        alu_func_r     = `ALU_AND4'b0111;
        alu_input_a_r  = opcode_ra_operand_i;
        alu_input_b_r  = opcode_rb_operand_i;
    end
    else if ((opcode_opcode_i & `INST_OR_MASK32'hfe00707f) == `INST_OR32'h6033) // or
    begin
        alu_func_r     = `ALU_OR4'b1000;
        alu_input_a_r  = opcode_ra_operand_i;
        alu_input_b_r  = opcode_rb_operand_i;
    end
    else if ((opcode_opcode_i & `INST_SLL_MASK32'hfe00707f) == `INST_SLL32'h1033) // sll
    begin
        alu_func_r     = `ALU_SHIFTL4'b0001;
        alu_input_a_r  = opcode_ra_operand_i;
        alu_input_b_r  = opcode_rb_operand_i;
    end
    else if ((opcode_opcode_i & `INST_SRA_MASK32'hfe00707f) == `INST_SRA32'h40005033) // sra
    begin
        alu_func_r     = `ALU_SHIFTR_ARITH4'b0011;
        alu_input_a_r  = opcode_ra_operand_i;
        alu_input_b_r  = opcode_rb_operand_i;
    end
    else if ((opcode_opcode_i & `INST_SRL_MASK32'hfe00707f) == `INST_SRL32'h5033) // srl
    begin
        alu_func_r     = `ALU_SHIFTR4'b0010;
        alu_input_a_r  = opcode_ra_operand_i;
        alu_input_b_r  = opcode_rb_operand_i;
    end
    else if ((opcode_opcode_i & `INST_SUB_MASK32'hfe00707f) == `INST_SUB32'h40000033) // sub
    begin
        alu_func_r     = `ALU_SUB4'b0110;
        alu_input_a_r  = opcode_ra_operand_i;
        alu_input_b_r  = opcode_rb_operand_i;
    end
    else if ((opcode_opcode_i & `INST_XOR_MASK32'hfe00707f) == `INST_XOR32'h4033) // xor
    begin
        alu_func_r     = `ALU_XOR4'b1001;
        alu_input_a_r  = opcode_ra_operand_i;
        alu_input_b_r  = opcode_rb_operand_i;
    end
    else if ((opcode_opcode_i & `INST_SLT_MASK32'hfe00707f) == `INST_SLT32'h2033) // slt
    begin
        alu_func_r     = `ALU_LESS_THAN_SIGNED4'b1011;
        alu_input_a_r  = opcode_ra_operand_i;
        alu_input_b_r  = opcode_rb_operand_i;
    end
    else if ((opcode_opcode_i & `INST_SLTU_MASK32'hfe00707f) == `INST_SLTU32'h3033) // sltu
    begin
        alu_func_r     = `ALU_LESS_THAN4'b1010;
        alu_input_a_r  = opcode_ra_operand_i;
        alu_input_b_r  = opcode_rb_operand_i;
    end
    else if ((opcode_opcode_i & `INST_ADDI_MASK32'h707f) == `INST_ADDI32'h13) // addi
    begin
        alu_func_r     = `ALU_ADD4'b0100;
        alu_input_a_r  = opcode_ra_operand_i;
        alu_input_b_r  = imm12_r;
    end
    else if ((opcode_opcode_i & `INST_ANDI_MASK32'h707f) == `INST_ANDI32'h7013) // andi
    begin
        alu_func_r     = `ALU_AND4'b0111;
        alu_input_a_r  = opcode_ra_operand_i;
        alu_input_b_r  = imm12_r;
    end
    else if ((opcode_opcode_i & `INST_SLTI_MASK32'h707f) == `INST_SLTI32'h2013) // slti
    begin
        alu_func_r     = `ALU_LESS_THAN_SIGNED4'b1011;
        alu_input_a_r  = opcode_ra_operand_i;
        alu_input_b_r  = imm12_r;
    end
    else if ((opcode_opcode_i & `INST_SLTIU_MASK32'h707f) == `INST_SLTIU32'h3013) // sltiu
    begin
        alu_func_r     = `ALU_LESS_THAN4'b1010;
        alu_input_a_r  = opcode_ra_operand_i;
        alu_input_b_r  = imm12_r;
    end
    else if ((opcode_opcode_i & `INST_ORI_MASK32'h707f) == `INST_ORI32'h6013) // ori
    begin
        alu_func_r     = `ALU_OR4'b1000;
        alu_input_a_r  = opcode_ra_operand_i;
        alu_input_b_r  = imm12_r;
    end
    else if ((opcode_opcode_i & `INST_XORI_MASK32'h707f) == `INST_XORI32'h4013) // xori
    begin
        alu_func_r     = `ALU_XOR4'b1001;
        alu_input_a_r  = opcode_ra_operand_i;
        alu_input_b_r  = imm12_r;
    end
    else if ((opcode_opcode_i & `INST_SLLI_MASK32'hfc00707f) == `INST_SLLI32'h1013) // slli
    begin
        alu_func_r     = `ALU_SHIFTL4'b0001;
        alu_input_a_r  = opcode_ra_operand_i;
        alu_input_b_r  = {27'b0, shamt_r};
    end
    else if ((opcode_opcode_i & `INST_SRLI_MASK32'hfc00707f) == `INST_SRLI32'h5013) // srli
    begin
        alu_func_r     = `ALU_SHIFTR4'b0010;
        alu_input_a_r  = opcode_ra_operand_i;
        alu_input_b_r  = {27'b0, shamt_r};
    end
    else if ((opcode_opcode_i & `INST_SRAI_MASK32'hfc00707f) == `INST_SRAI32'h40005013) // srai
    begin
        alu_func_r     = `ALU_SHIFTR_ARITH4'b0011;
        alu_input_a_r  = opcode_ra_operand_i;
        alu_input_b_r  = {27'b0, shamt_r};
    end
    else if ((opcode_opcode_i & `INST_LUI_MASK32'h7f) == `INST_LUI32'h37) // lui
    begin
        alu_input_a_r  = imm20_r;
    end
    else if ((opcode_opcode_i & `INST_AUIPC_MASK32'h7f) == `INST_AUIPC32'h17) // auipc
    begin
        alu_func_r     = `ALU_ADD4'b0100;
        alu_input_a_r  = opcode_pc_i;
        alu_input_b_r  = imm20_r;
    end     
    else if (((opcode_opcode_i & `INST_JAL_MASK32'h7f) == `INST_JAL32'h6f) || ((opcode_opcode_i & `INST_JALR_MASK32'h707f) == `INST_JALR32'h67)) // jal, jalr
    begin
        alu_func_r     = `ALU_ADD4'b0100;
        alu_input_a_r  = opcode_pc_i;
        alu_input_b_r  = 32'd4;
    end
end
//-------------------------------------------------------------
// ALU
//-------------------------------------------------------------
wire [31:0]  alu_p_w;
riscv_alu
u_alu
(
    .alu_op_i(alu_func_r),
    .alu_a_i(alu_input_a_r),
    .alu_b_i(alu_input_b_r),
    .alu_p_o(alu_p_w)
);
//-------------------------------------------------------------
// Flop ALU output
//-------------------------------------------------------------
reg [31:0] result_q;
always @ (posedge clk_i or posedge rst_i)
if (rst_i)
    result_q  <= 32'b0;
else if (~hold_i)
    result_q <= alu_p_w;
assign writeback_value_o  = result_q;
//-----------------------------------------------------------------
// less_than_signed: Less than operator (signed)
// Inputs: x = left operand, y = right operand
// Return: (int)x < (int)y
//-----------------------------------------------------------------
function [0:0] less_than_signed;
    input  [31:0] x;
    input  [31:0] y;
    reg [31:0] v;
begin
    v = (x - y);
    if (x[31] != y[31])
        less_than_signed = x[31];
    else
        less_than_signed = v[31];
end
endfunction
//-----------------------------------------------------------------
// greater_than_signed: Greater than operator (signed)
// Inputs: x = left operand, y = right operand
// Return: (int)x > (int)y
//-----------------------------------------------------------------
function [0:0] greater_than_signed;
    input  [31:0] x;
    input  [31:0] y;
    reg [31:0] v;
begin
    v = (y - x);
    if (x[31] != y[31])
        greater_than_signed = y[31];
    else
        greater_than_signed = v[31];
end
endfunction
//-------------------------------------------------------------
// Execute - Branch operations
//-------------------------------------------------------------
reg        branch_r;
reg        branch_taken_r;
reg [31:0] branch_target_r;
reg        branch_call_r;
reg        branch_ret_r;
reg        branch_jmp_r;
always @ *
begin
    branch_r        = 1'b0;
    branch_taken_r  = 1'b0;
    branch_call_r   = 1'b0;
    branch_ret_r    = 1'b0;
    branch_jmp_r    = 1'b0;
    // Default branch_r target is relative to current PC
    branch_target_r = opcode_pc_i + bimm_r;
    if ((opcode_opcode_i & `INST_JAL_MASK32'h7f) == `INST_JAL32'h6f) // jal
    begin
        branch_r        = 1'b1;
        branch_taken_r  = 1'b1;
        branch_target_r = opcode_pc_i + jimm20_r;
        branch_call_r   = (opcode_rd_idx_i == 5'd1); // RA
        branch_jmp_r    = 1'b1;
    end
    else if ((opcode_opcode_i & `INST_JALR_MASK32'h707f) == `INST_JALR32'h67) // jalr
    begin
        branch_r            = 1'b1;
        branch_taken_r      = 1'b1;
        branch_target_r     = opcode_ra_operand_i + imm12_r;
        branch_target_r[0]  = 1'b0;
        branch_ret_r        = (opcode_ra_idx_i == 5'd1 && imm12_r[11:0] == 12'b0); // RA
        branch_call_r       = ~branch_ret_r && (opcode_rd_idx_i == 5'd1); // RA
        branch_jmp_r        = ~(branch_call_r | branch_ret_r);
    end
    else if ((opcode_opcode_i & `INST_BEQ_MASK32'h707f) == `INST_BEQ32'h63) // beq
    begin
        branch_r      = 1'b1;
        branch_taken_r= (opcode_ra_operand_i == opcode_rb_operand_i);
    end
    else if ((opcode_opcode_i & `INST_BNE_MASK32'h707f) == `INST_BNE32'h1063) // bne
    begin
        branch_r      = 1'b1;    
        branch_taken_r= (opcode_ra_operand_i != opcode_rb_operand_i);
    end
    else if ((opcode_opcode_i & `INST_BLT_MASK32'h707f) == `INST_BLT32'h4063) // blt
    begin
        branch_r      = 1'b1;
        branch_taken_r= less_than_signed(opcode_ra_operand_i, opcode_rb_operand_i);
    end
    else if ((opcode_opcode_i & `INST_BGE_MASK32'h707f) == `INST_BGE32'h5063) // bge
    begin
        branch_r      = 1'b1;    
        branch_taken_r= greater_than_signed(opcode_ra_operand_i,opcode_rb_operand_i) | (opcode_ra_operand_i == opcode_rb_operand_i);
    end
    else if ((opcode_opcode_i & `INST_BLTU_MASK32'h707f) == `INST_BLTU32'h6063) // bltu
    begin
        branch_r      = 1'b1;    
        branch_taken_r= (opcode_ra_operand_i < opcode_rb_operand_i);
    end
    else if ((opcode_opcode_i & `INST_BGEU_MASK32'h707f) == `INST_BGEU32'h7063) // bgeu
    begin
        branch_r      = 1'b1;
        branch_taken_r= (opcode_ra_operand_i >= opcode_rb_operand_i);
    end
end
reg        branch_taken_q;
reg        branch_ntaken_q;
reg [31:0] pc_x_q;
reg [31:0] pc_m_q;
reg        branch_call_q;
reg        branch_ret_q;
reg        branch_jmp_q;
always @ (posedge clk_i or posedge rst_i)
if (rst_i)
begin
    branch_taken_q   <= 1'b0;
    branch_ntaken_q  <= 1'b0;
    pc_x_q           <= 32'b0;
    pc_m_q           <= 32'b0;
    branch_call_q    <= 1'b0;
    branch_ret_q     <= 1'b0;
    branch_jmp_q     <= 1'b0;
end
else if (opcode_valid_i)
begin
    branch_taken_q   <= branch_r && opcode_valid_i & branch_taken_r;
    branch_ntaken_q  <= branch_r && opcode_valid_i & ~branch_taken_r;
    pc_x_q           <= branch_taken_r ? branch_target_r : opcode_pc_i + 32'd4;
    branch_call_q    <= branch_r && opcode_valid_i && branch_call_r;
    branch_ret_q     <= branch_r && opcode_valid_i && branch_ret_r;
    branch_jmp_q     <= branch_r && opcode_valid_i && branch_jmp_r;
    pc_m_q           <= opcode_pc_i;
end
assign branch_request_o   = branch_taken_q | branch_ntaken_q;
assign branch_is_taken_o  = branch_taken_q;
assign branch_is_not_taken_o = branch_ntaken_q;
assign branch_source_o    = pc_m_q;
assign branch_pc_o        = pc_x_q;
assign branch_is_call_o   = branch_call_q;
assign branch_is_ret_o    = branch_ret_q;
assign branch_is_jmp_o    = branch_jmp_q;
assign branch_d_request_o = (branch_r && opcode_valid_i && branch_taken_r);
assign branch_d_pc_o      = branch_target_r;
assign branch_d_priv_o    = 2'b0; // don't care
endmodule

