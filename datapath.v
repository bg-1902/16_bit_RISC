//
//
//

`timescale 1ns/1ps
`define STOPTIME 661

module tb_multi_cycle();

    reg clk, rst;

    initial begin
        clk = 1'b0;
        rst = 1'b1;
        #3 rst = 1'b0;
    end

    MIPS uut    (   .clk(clk),
                    .rst(rst));
    
    always #2 clk = ~clk;

    initial begin
        $dumpfile("output.vcd");
        $dumpvars;
        #`STOPTIME $finish;
    end

    // Defining wires for testbench from uut, done only for ease of access purposes
    wire [15:0] Instruction;
    wire [4:0] State;
    assign Instruction = uut.Datapath.IR_out;
    assign State = uut.ControlUnit.State;
    
    // Count to track instruction numbers
    integer count = 0;
    
    always @(negedge clk) begin
        $timeformat(-9, 0, "ns", 5);
        
        // IF States
        if(State == 4'd0) begin
            count = count + 1;
            #1;
            $write("\n[T = %0t]\t#%2d Instruction: %h ",$time, count, Instruction);
            
            if      (Instruction[15:12] == 4'b0100)  $write("\t(BEQ)");
            else if (Instruction[15:12] == 4'b0101)  $write("\t(BNEQ)");
            else if (Instruction[15:12] == 4'b0011)  $write("\t(Jump)");
            else if (Instruction[15:12] == 4'b0001)  $write("\t(Load)");
            else if (Instruction[15:12] == 4'b0010)  $write("\t(Store)");
            else                                     $write("\t(R type)");

            $write("\nState: IF");
            $write("\tPC: %hH", uut.Datapath.PC_out);
            $write("\tIR: %hH", uut.Datapath.IR_out);
        end
        
        // ID States
        else if(State == 4'd1 || State == 4'd2 || State == 4'd3) begin
            $write("State: ID");
            #1;
            $write("\tA (R%0d): %hH\tB (R%0d): %hH\tC (R%0d): %hH", uut.Datapath.RegisterFile.ReadReg1, uut.Datapath.A_out, uut.Datapath.RegisterFile.ReadReg2, uut.Datapath.B_out, uut.Datapath.RegisterFile.ReadReg3, uut.Datapath.C_out);
        end
        
        // EX States
        else if(State == 4'd4 || State == 4'd5 || State == 4'd6 || State == 4'd7 ||State == 4'd8 ||State == 4'd9 || State == 4'd10 || State == 4'd11 ||State == 4'd12 ||State == 4'd13 ||State == 4'd14 || State == 4'd15 || State == 4'd16 || State == 4'd17 || State == 4'd18 || State == 4'd19 || State == 4'd20 || ) begin
            #1;
            $write("State: EX");
            
            // Branch
            if(Instruction[15:12] == 4'b0100 || Instruction[15:12] == 4'b0101)
                $write("\tZero: %hH\tPC: %hH\tBranch Taken: %b", uut.Datapath.Zero, uut.Datapath.PC_out, uut.Datapath.PCWrite_in);
            
            
            // Jump
            else if(Instruction[15:12] == 4'b0011) begin
                $write("\tPC: %hH", uut.Datapath.PC_out);
            end
            
            // R-type + LW/SW
            else begin
                $write("\tALUOut: %hH", uut.Datapath.ALUOut_out);
                $write("\tALU Operation Done: %hH", uut.Datapath.ALUSrcA_m_out);
                case(uut.Datapath.ALU.ALUOp)
                    3'b000: $write(" Add");      //addition
		            3'b001: $write(" Subtract"); //subtraction
		            3'b010: $write(" NAND");     //NAND
		            3'b011: $write(" OR");       // OR
		            3'b100: $write(" SLL");      // SLL 
		            3'b101: $write(" SRL");      // SRL
		            3'b110: $write(" SAR");      // SAR
                endcase
                $write(" %hH", uut.Datapath.ALUSrcB_m_out);
            end
        end
        
        // MEM States
        else if(State == 4'd21 || State == 4'd22 || State == 4'd23) begin
            $write("State: MEM");
            // Load
            if(Instruction[15:12] == 4'b0001) begin
                $write("\tMemory[ALUOut]: %hH", uut.Datapath.DataMem.Memory[uut.Datapath.DataMem.ActualAddress]);
                #1 $write("\tMDR: %hH", uut.Datapath.MDR_out);
            end
            // Store
            else if(Instruction[15:12] == 4'b0010)
                #1 $write("\tMemory[ALUOut]: %hH\tB: %hH", uut.Datapath.DataMem.Memory[uut.Datapath.DataMem.Address[15:1]], uut.Datapath.B_out);
            // R-type
            else begin
                #1;
                $write("\tRD: R%0d\tReg[RD]: %hH", uut.Datapath.RFile.WriteRegister, uut.Datapath.RFile.Registers[uut.Datapath.RFile.WriteRegister]);
            end
        end
        
        // WB States
        else begin
            #1;
            $write("State: WB");
            $write("\t11RD: R%0dH\tReg[11RD]: %h", {2'b11, uut.Datapath.RD}, uut.Datapath.RFile.Registers[{2'b11, uut.Datapath.RD_l}]);
        
        end
        $write("\nControl Signals |\t.clk: %b,
                        .PCWrite: %b,
                        .PCWriteCond: %b,
                        .BNEq: %b,
                        .MemRd: %b,
                        .MemWr: %b,
                        .IRd: %b,
                        .IRWr: %b,
                        .RegWrite: %b,
                        .PCWrite_in: %b,
                        .RegDst: %b,
                        .MemToReg: %b,
                        .SESF: %b,
                        .JE: %b,
                        .ALUSrcA: %b,
                        .R1Src: %b,
                        .ALUSrcB: %b,
                        .PCSrc: %b,
                        .ALUCtrltrl: %b,
                        .OpCodede: %b,
                        .Func: %b", uut.clk,
                        uut.PCWrite,
                        uut.PCWriteCond,
                        uut.BNEq,
                        uut.MemRd,
                        uut.MemWr,
                        uut.IRd,
                        uut.IRWr,
                        uut.RegWrite,
                        uut.PCWrite_in,
                        uut.RegDst,
                        uut.MemToReg,
                        uut.SESF,
                        uut.JE,
                        uut.ALUSrcA,
                        uut.R1Src,
                        uut.ALUSrcB,
                        uut.PCSrc,
                        uut.ALUCtrltrl,
                        uut.OpCodede,
                        uut.Func)
        
        $write("\n");
    end

endmodule

module alu(
    input [15:0] A,B,                  
    input [2:0] ALU_Sel,
    output [15:0] ALU_Out,
    output Zero;
    );

    reg ZF;
    reg [15:0] ALU_Result;
    assign ALU_Out = ALU_Result; // ALU out
    assign Zero = ZF;

    always @(*)
    begin
        case(ALU_Sel)
        3'b000: // Addition
            ALU_Result = A + B ; 
        3'b001: // Subtraction
            ALU_Result = A - B ;
        3'b010: // SLL
            ALU_Result = A << B;
        3'b011: // SLR
            ALU_Result = A >> B;
        3'b100: // SAR
            ALU_Result = $signed(A) >>> B ;
        3'b101: // NAND
            ALU_Result = ~(A & B);
        3'b110: // OR
            ALU_Result = A | B;
        3'b111: // ??
            ALU_Result = 16'bz;
        default: 
            ALU_Result = 16'bz; 
        endcase

        if (ALU_Result == 16'd0)
        begin
            ZF = 1'b1;
        end
        else
        begin
            ZF = 1'b0;
        end
    end

endmodule

module SEx_8to16 (ext, unext);
    output reg [15:0] ext;
    input [7:0] unext;

    always@(*)
    begin 
        ext <= $signed(unext);
    end
endmodule

module SEx_12to16 (ext, unext);
    output reg [15:0] ext;
    input [11:0] unext;

    always@(*)
    begin 
        ext <= $signed(unext);
    end
endmodule

module ZP_8to16 (ext, unext);
    output reg [15:0] ext;
    input [7:0] unext;

    always@(*)
    begin 
        ext <= $unsigned(unext);
    end
endmodule

module LeftShift (Output, Input);
    output reg [15:0] Output;
    input [15:0] Input;

    always @(*) 
    begin
        Output <= {Input[15:1], 1'b0};
    end
endmodule

module instr_mem(
    input clk;
    input IRd;
    input[15:0] pc,

    output[15:0] instruction
);

    reg [15:0] memory [0:32767];
    wire [14 : 0] address = pc[15 : 1];

    initial begin
        $readmemh("instruction.dat", Memory);
    end

    always @(*) begin
        if(IRd == 1'b1) begin
            instruction <= memory[address];
        end
    end

endmodule

module data_mem(
    input clk,
    input[15:0] Address,
    input [15:0] WriteData,
    input MemRead, MemWrite,
    output[15:0] MemData;
);

    input clk;
    input [15:0] Address;
    input [15:0] WriteData;
    input MemRead, MemWrite;

    reg [14:0] ActualAddress;

    output reg [15:0] MemData;

    always @(*) begin
        ActualAddress <= Address[15:1];
    end
    // 16-bit x 32K locations
    // !NOTE: The LSB of Address is dropped, therefore only aligned data is read
    reg [15:0] Memory [0:(32*1024) - 1];

    initial begin
        $readmemh("data.dat", Memory);
        #`STOPTIME $writememh("data.dat", Memory);
    end

    always @(*) begin
        if(MemRead == 1'b1) begin
            MemData <= Memory[ActualAddress];
        end
    end

    always @(negedge clk) begin
        if(MemWrite == 1'b1) begin
            Memory[ActualAddress] <= WriteData;
        end
    end
    
endmodule

module reg_16_bit(clk, Output, Input, Write, rst);

    input clk, Write, rst;
    input [15:0] Input;

    output reg [15:0] Output;
    
    initial begin
        Output <= 16'd0;
    end

    always @(negedge clk) begin
        if(rst == 1'b1) begin
            Output <= 16'd0;
        end
        else if(Write == 1'b1)
            Output <= Input;
    end

endmodule

module regFile(clk, RegWrite, ReadReg1, ReadReg2, ReadReg3, WriteRegister, WriteData, ReadData1, ReadData2, ReadData3);

    input clk;
    input RegWrite;
    input [15:0] WriteData;
    input [3:0] ReadReg1, ReadReg2, ReadReg3, WriteRegister;

    output reg [15:0] ReadData1, ReadData2, ReadData3;

    reg [15:0] Registers [0:15];

    initial begin
        Registers[0] <= 16'd1;
        // #`STOPTIME $writememh("registers.dat", Registers);
    end


    always @(posedge clk) begin
        if(RegWrite == 1'b1) begin
            if(WriteRegister == 4'd0) begin
                Registers[0] <= 16'd0;
            end
            else begin
                Registers[WriteRegister] <= WriteData;
            end
        end
    end

    always @(*) begin
        ReadData1 <= Registers[ReadReg1];
        ReadData2 <= Registers[ReadReg2];
        ReadData3 <= Registers[ReadReg3];
    end

endmodule

module Mux_2to1_4 (Output, Input0, Input1, Select);

    input Select;
    input [3:0] Input0, Input1;

    output reg [3:0] Output;
    initial begin
            Output <= 4'd0;
    end
    always @(*) begin
       if(Select == 1'b0)  Output <= Input0;
       else if(Select == 1'b1)    Output <= Input1;
       else Output <= 4'bxxxx;
    end

endmodule

module Mux_2to1_16 (Output, Input0, Input1, Select);

    input Select;
    input [15:0] Input0, Input1;

    output reg [15:0] Output;
    initial begin
        Output <= 16'd0;
    end
    always @(*) begin
       if(Select == 1'b0)  Output <= Input0;
       else if(Select == 1'b1)    Output <= Input1;
       else Output <= 16'bxxxxxxxxxxxxxxxx;
    end

endmodule

module Mux_4to1_4(Output, Input0, Input1, Input2, Input3, Select);
    input [1:0] Select;
    input [3:0] Input0, Input1, Input2, Input3;

    output reg [3:0] Output;
    initial begin
        Output <= 4'd0;
    end
    always @(*) begin
       if(Select == 2'b00)  Output <= Input0;
       else if(Select == 2'b01)    Output <= Input1;
       else if(Select == 2'b10)    Output <= Input2;
       else if(Select == 2'b11)    Output <= Input3;
       else Output <= 4'bxxxx;
    end

endmodule

module Mux_4to1_16(Output, Input0, Input1, Input2, Input3, Select);
    input [1:0] Select;
    input [15:0] Input0, Input1, Input2, Input3;

    output reg [15:0] Output;
    initial begin
        Output <= 16'd0;
    end
    always @(*) begin
       if(Select == 2'b00)  Output <= Input0;
       else if(Select == 2'b01)    Output <= Input1;
       else if(Select == 2'b10)    Output <= Input2;
       else if(Select == 2'b11)    Output <= Input3;
       else Output <= 16'bxxxx;
    end

endmodule

module Datapath(clk, rst, PCWrite, PCWriteCond, BNEq, MemRd, MemWr, IRd, IRWr, RegWrite, PCWrite_in, RegDst, MemToReg, SESF, JE, ALUSrcA, R1Src, ALUSrcB, PCSrc, ALUCtrl, OpCode, Func);
    input clk;
    input rst;

    //PC signals
    input wire PCWrite;
    input wire PCWriteCond;
    input wire BNEq;
    
    //mem
    input wire MemRd;
    input wire MemWr;
    input wire IRd;
    input wire IRWr;

    //rf
    input wire RegWrite;
    input wire PCWrite_in;

    // 2:1 Mux Control Signals
    input wire RegDst;
    input wire MemToReg;
    input wire SESF;
    input wire JE;
    input wire ALUSrcA;

    // 4:1 Mux Control Signals
    input wire [1:0] R1Src;
    input wire [1:0] ALUSrcB;
    input wire [1:0] PCSrc;

    //ALU
    input wire [2:0] ALUCtrl;

    output wire [3:0] OpCode;
    output wire [3:0] Func;

    //Internal Wires

    // Output Zero of ALU
    wire Zero;
    
    // Outputs of registers
    wire [15:0] PC_out; 
    wire [15:0] ALUOut_out; 
    wire [15:0] IR_out;
    wire [15:0] MDR_out;
    wire [15:0] A_out; 
    wire [15:0] B_out; 
    wire [15:0] C_out;

    // Outputs of memory blocks
    wire [15:0] DataMem_out, InstrMem_out;

    // Outputs of Register File
    wire [15:0] ReadData1;
    wire [15:0] ReadData2;
    wire [15:0] ReadData3;

    // Outputs of 2:1 mux blocks (4 bit data)
    wire [3:0] RegDst_m_out;

    // Outputs of 2:1 mux blocks (16 bit data)
    wire [15:0] SESF_m_out;
    wire [15:0] JE_m_out;
    wire [15:0] MemToReg_m_out; 
    wire [15:0] ALUSrcA_m_out;

    // Outputs of 4:1 Mux Control Signals (4 bit data)
    wire [3:0] R1Src_m_out;

    // Outputs of 4:1 Mux Control Signals (16 bit data)
    wire [15:0] ALUSrcB_m_out;
    wire [15:0] PCSrc_m_out;

    // Output of ALU
    wire [15:0] ALU_out;

    // Outputs of extenders
    wire [15:0] ZP_8to16_out;
    wire [15:0] SEx_12to16_out;
    wire [15:0] SEx_8to16_out;
    wire [15:0] LeftShift_out;

    wire [3:0] Opcode, RA, RB, RC;
    wire [1:0] RD_l, RP_l;

    wire Branch;
    wire ChangePC;

    buf buf1 [15:0]({Opcode, RA, RB, RC},IR_out);
    buf buf2 [3:0]({RD_l, RP_l}, IR_out[11:8]);

    xnor xnor1(Branch, Zero, BranchEq);
    and and1 (  ChangePC, Branch, PCWriteCond);
    or or1   (  PCWrite_in, ChangePC, PCWrite);

    reg_16_bit PC (.clk(clk),
                    .Output(PC_out),
                    .Input(PCSrc_m_out),
                    .Write(PCWrite_in),
                    .rst(rst));
    
    reg_16_bit IR (.clk(clk),
                    .Output(IR_out),
                    .Input(InstrMem_out),
                    .Write(IRWr),
                    .rst(rst));

    reg_16_bit MDR (   .clk(clk),
                        .Output(MDR_out),
                        .Input(DataMem_out),
                        .Write(1'b1),
                        .rst(rst));

    reg_16_bit A ( .clk(clk),
                    .Output(A_out),
                    .Input(ReadData1),
                    .Write(1'b1),
                    .rst(rst));
    reg_16_bit B ( .clk(clk),
                    .Output(B_out),
                    .Input(ReadData2),
                    .Write(1'b1),
                    .rst(rst));
    reg_16_bit C ( .clk(clk),
                    .Output(C_out),
                    .Input(ReadData3),
                    .Write(1'b1),
                    .rst(rst));

    reg_16_bit ALUOut (    .clk(clk),
                            .Output(ALUOut_out),
                            .Input(ALU_out),
                            .Write(1'b1),
                            .rst(rst));


    instr_mem InstrMem (    .clk(clk),
                                    .pc(PC_out),
                                    .instruction(InstrMem_out),
                                    .IRd(IRd));

    data_mem DataMem (    .clk(clk),
                            .Address(ALUOut_out),
                            .WriteData(C_out),
                            .MemData(DataMem_out),
                            .MemRead(MemRd),
                            .MemWrite(MemWr));

    regFile RFile ( .clk(clk),
                                .RegWrite(RegWr),
                                .ReadReg1(R1Src_m_out),
                                .ReadReg2(RC),
                                .ReadReg3(RegDst_m_out),
                                .WriteRegister(RegDst_m_out),
                                .WriteData(MemToReg_m_out),
                                .ReadData1(ReadData1),
                                .ReadData2(ReadData2),
                                .ReadData3(ReadData3));

    alu ALU (   .A(ALUSrcA_m_out),
                .B(ALUSrcB_m_out),
                .ALU_Sel(ALUCtrl),
                .ALU_Out(ALU_out),
                .Zero(Zero));

    Mux_2to1_4 RegDst_m(    .Output(RegDst_m_out),
                            .Input0(RA),
                            .Input1({2'b11, RD_l}),
                            .Select(RegDst));

    Mux_2to1_16 SESF_m( .Output(SESF_m_out),
                        .Input0(ZP_8to16_out),
                        .Input1(SEx_8to16_out),
                        .Select(SESF));

    Mux_2to1_16 JE_m(   .Output(JE_m_out),
                        .Input0(SESF_m_out),
                        .Input1(SEx_12to16_out),
                        .Select(JE));

    Mux_2to1_16 MemToReg_m(     .Output(MemToReg_m_out),
                                .Input0(ALUOut_out),
                                .Input1(MDR_out),
                                .Select(MemToReg));

    Mux_2to1_16 ALUSrcA_m(  .Output(ALUSrcA_m_out),
                            .Input0(PC_out),
                            .Input1(A_out),
                            .Select(ALUSrcA));

    Mux_4to1_4 R1Src_m(     .Output(R1Src_m_out),
                            .Input0({2'b10, RP_l}),
                            .Input1(RB),
                            .Input2(RA),
                            .Input3(4'bx),
                            .Select(R1Src));

    Mux_4to1_16 ALUSrcB_m(  .Output(ALUSrcB_m_out),
                            .Input0(B_out),
                            .Input1(16'd2),
                            .Input2(JE_m_out),
                            .Input3(LeftShift_out),
                            .Select(ALUSrcB));

    Mux_4to1_16 PCSrc_m(.Output(PCSrc_m_out),
                            .Input0(ALUOut_out),
                            .Input1(C_out),
                            .Input2(ALU_out),
                            .Input3(16'bx),
                            .Select(PCSrc));

    SEx_8to16 sex1(.ext(SEx_8to16_out),
                     .unext({RB, RC}));

    SEx_12to16 sex2(.ext(SEx_12to16_out), 
                    .unext({RA, RB, RC}));

    ZP_8to16 zp1(.ext(ZP_8to16_out), 
                .unext({RB, RC}));

    LeftShift ls1(.Output(LeftShift_out), 
                    .Input(SEx_8to16_out));
endmodule


// ! posedge clk
module Control(clk, rst, OpCode, Func, IRd, ALUSrcA, ALUSrcB, PCWrite, PCSrc, R1Src, R2Src, SESF, PCWriteCond, MemRd, MemWr, MemToReg, ALU, RegWr, RegDst, BNEq, JE);
    
    input clk;
    input rst;
    input [3:0] OpCode;
    input [3:0] Func;
    
    output reg IRd, ALUSrcA, PCWrite, PCSrc, R2Src, SESF, PCWriteCond, MemRd, MemWr, MemToReg, RegWr, RegDst, JE, BNEq;
    output reg [1:0] ALUSrcB,R1Src;
    output reg [2:0] ALU; 

    reg [4:0] State, NextState;

    always @(posedge clk) begin
        if(rst == 1'b1) begin
            State <= 4'd0;
        end
        else State <= NextState;
    end

    always @(*) begin
        case (State)
        5'd0: begin
                if(OpCode == 4'b1000 || OpCode == 4'b1100 || OpCode == 4'b1011 || OpCode == 4'b1111 || OpCode == 4'b0100 || OpCode == 4'b0101 || OpCode == 4'b0011) NextState <= 4'd1;
                else if(OpCode == 4'b1001 || OpCode == 4'b1101 || OpCode == 4'b0111 || OpCode == 4'b0110 || OpCode == 4'b1010 || OpCode == 4'b1110 ||OpCode == 4'b0000) NextState <= 4'd2;
                else NextState <= 4'd3;
            end
        5'd1: begin
                if(OpCode == 4'b1111)   NextState <= 4'd4;
                else if(OpCode == 4'b1011)  NextState <= 4'd5;
                else if(OpCode == 4'b1000)  NextState <= 4'd6;
                else if(OpCode == 4'b1100)  NextState <= 4'd7;
                else if(OpCode == 4'b0100)  NextState <= 4'd8;
                else if(OpCode == 4'b0101)  NextState <= 4'd9;
                else if(OpCode == 4'b0011)  NextState <= 4'd20;
            end
        5'd2: begin
                if(OpCode == 4'b1010)   NextState <= 4'd10;
                else if(OpCode == 4'b1001)  NextState <= 4'd11;
                else if(OpCode == 4'b1110)  NextState <= 4'd12;
                else if(OpCode == 4'b1101)  NextState <= 4'd13;
                else if(OpCode == 4'b0110)  NextState <= 4'd14;
                else if(OpCode == 4'b0111)  NextState <= 4'd15;
                else if(OpCode == 4'b0000 && Func = 4'b0001)  NextState <= 4'd16;
                else if(OpCode == 4'b0000 && Func = 4'b0010)  NextState <= 4'd17;
                else if(OpCode == 4'b0000 && Func = 4'b0011)  NextState <= 4'd18;
                
            end
            end
        5'd3: begin
                NextState <= 4'd19;
            end
        5'd4: begin
                NextState <= 4'd21;
            end
        5'd5: begin
                NextState <= 4'd21;
            end
        5'd6: begin
                NextState <= 4'd21;
            end
        5'd7: begin
                NextState <= 4'd21;
            end
        5'd8: begin
                NextState <= 4'd0;
            end
        5'd9: begin
                NextState <= 4'd0;
            end
            5'd10: begin
                NextState <= 4'd21;
            end
            5'd11: begin
                NextState <= 4'd21;
            end
            5'd12: begin
                NextState <= 4'd21;
            end
            5'd13: begin
                NextState <= 4'd21;
            end
            5'd14: begin
                NextState <= 4'd21;
            end
            5'd15: begin
                NextState <= 4'd21;
            end
            5'd16: begin
                NextState <= 4'd21;
            end
            5'd17: begin
                NextState <= 4'd21;
            end
            5'd18: begin
                NextState <= 4'd21;
            end
            5'd19: begin
                if(OpCode == 4'b0001)   NextState <= 4'd22;
                else if(OpCode == 4'b0010)  NextState <= 4'd23;
            end
            5'd20: begin
                NextState <= 4'd0;
            end
            5'd21: begin
                NextState <= 4'd0;
            end
            5'd22: begin
                NextState <= 4'd24;
            end
            5'd23: begin
                NextState <= 4'd0;
            end
            5'd24: begin
                NextState <= 4'd0;
            end
        endcase
    end

    initial begin
        State <= 4'd0;
    end

    always @(State) begin
        case (State)
        5'd0: begin
                IRd <= 1'b1;
                ALUSrcA <= 1'b0;
                ALUSrcB <= 2'b01;
                PCWrite <= 1'b1;
                PCSrc <= 1'b0;
                R1Src <= 2'b00;
                R2Src <= 1'b0;
                SESF <= 1'b0;
                PCWriteCond <= 1'b0;
                MemRd <= 1'b0;
                MemWr <= 1'b0;
                MemToReg <= 1'b0;
                ALU <= 3'b000;
                RegWr <= 1'b0;
                RegDst <= 1'b0;
                BNEq <= 1'b0;
                JE <= 1'b0;
            end
        5'd1: begin
                // IRd <= 1'b0;
                // ALUSrcA <= 1'b0;
                // ALUSrcB <= 2'b01;
                // PCWrite <= 1'b0;
                // PCSrc <= 1'b0;
                // R1Src <= 2'b01;
                // R2Src <= 1'b0;
                // SESF <= 1'b0;
                // PCWriteCond <= 1'b0;
                // MemRd <= 1'b0;
                // MemWr <= 1'b0;
                // MemToReg <= 1'b0;
                // ALU <= 3'b000;
                // RegWr <= 1'b0;
                // RegDst <= 1'b0;
                // BNEq <= 1'b0;
                // JE <= 1'b1;
                IRd <= 1'b0;
                PCWrite <= 1'b0;
                R1Src <= 2'b01;
                JE <= 1'b1;
            end
        5'd2: begin
                // IRd <= 1'b0;
                // ALUSrcA <= 1'b0;
                // ALUSrcB <= 2'b01;
                // PCWrite <= 1'b0;
                // PCSrc <= 1'b0;
                // R1Src <= 2'b10;
                // R2Src <= 1'b0;
                // SESF <= 1'b0;
                // PCWriteCond <= 1'b0;
                // MemRd <= 1'b0;
                // MemWr <= 1'b0;
                // MemToReg <= 1'b0;
                // ALU <= 3'b000;
                // RegWr <= 1'b0;
                // RegDst <= 1'b0;
                // BNEq <= 1'b0;
                // JE <= 1'b0;
                IRd <= 1'b0;
                PCWrite <= 1'b0;
                R1Src <= 2'b10;
            end
        5'd3: begin
                // IRd <= 1'b0;
                // ALUSrcA <= 1'b0;
                // ALUSrcB <= 2'b01;
                // PCWrite <= 1'b0;
                // PCSrc <= 1'b0;
                // R1Src <= 2'b00;
                // R2Src <= 1'b0;
                // SESF <= 1'b0;
                // PCWriteCond <= 1'b0;
                // MemRd <= 1'b0;
                // MemWr <= 1'b0;
                // MemToReg <= 1'b0;
                // ALU <= 3'b000;
                // RegWr <= 1'b0;
                // RegDst <= 1'b0;
                // BNEq <= 1'b0;
                // JE <= 1'b0;
                IRd <= 1'b0;
                PCWrite <= 1'b0;
                R1Src <= 2'b00;
            end
        5'd4: begin
                // IRd <= 1'b0;
                // ALUSrcA <= 1'b1;
                // ALUSrcB <= 2'b00;
                // PCWrite <= 1'b0;
                // PCSrc <= 1'b0;
                // R1Src <= 2'b00;
                // R2Src <= 1'b0;
                // SESF <= 1'b0;
                // PCWriteCond <= 1'b0;
                // MemRd <= 1'b0;
                // MemWr <= 1'b0;
                // MemToReg <= 1'b0;
                // ALU <= 3'b110;
                // RegWr <= 1'b0;
                // RegDst <= 1'b0;
                // BNEq <= 1'b0;
                // JE <= 1'b0;
                ALUSrcA <= 1'b1;
                ALUSrcB <= 2'b00;
                ALU <= 3'b110;
            end
        5'd5: begin
                // IRd <= 1'b0;
                // ALUSrcA <= 1'b1;
                // ALUSrcB <= 2'b00;
                // PCWrite <= 1'b0;
                // PCSrc <= 1'b0;
                // R1Src <= 2'b00;
                // R2Src <= 1'b0;
                // SESF <= 1'b0;
                // PCWriteCond <= 1'b0;
                // MemRd <= 1'b0;
                // MemWr <= 1'b0;
                // MemToReg <= 1'b0;
                // ALU <= 3'b101;
                // RegWr <= 1'b0;
                // RegDst <= 1'b0;
                // BNEq <= 1'b0;
                // JE <= 1'b0;
                ALUSrcA <= 1'b1;
                ALUSrcB <= 2'b00;
                ALU <= 3'b101;
            end
        5'd6: begin
                // IRd <= 1'b0;
                // ALUSrcA <= 1'b1;
                // ALUSrcB <= 2'b00;
                // PCWrite <= 1'b0;
                // PCSrc <= 1'b0;
                // R1Src <= 2'b00;
                // R2Src <= 1'b0;
                // SESF <= 1'b0;
                // PCWriteCond <= 1'b0;
                // MemRd <= 1'b0;
                // MemWr <= 1'b0;
                // MemToReg <= 1'b0;
                // ALU <= 3'b000;
                // RegWr <= 1'b0;
                // RegDst <= 1'b0;
                // BNEq <= 1'b0;
                // JE <= 1'b0;
                ALUSrcA <= 1'b1;
                ALUSrcB <= 2'b00;
                ALU <= 3'b000;
            end
        5'd7: begin
                // IRd <= 1'b0;
                // ALUSrcA <= 1'b1;
                // ALUSrcB <= 2'b00;
                // PCWrite <= 1'b0;
                // PCSrc <= 1'b0;
                // R1Src <= 2'b00;
                // R2Src <= 1'b0;
                // SESF <= 1'b0;
                // PCWriteCond <= 1'b0;
                // MemRd <= 1'b0;
                // MemWr <= 1'b0;
                // MemToReg <= 1'b0;
                // ALU <= 3'b001;
                // RegWr <= 1'b0;
                // RegDst <= 1'b0;
                // BNEq <= 1'b0;
                // JE <= 1'b0;
                ALUSrcA <= 1'b1;
                ALUSrcB <= 2'b00;
                ALU <= 3'b001;
            end
        5'd8: begin
                // IRd <= 1'b0;
                // ALUSrcA <= 1'b1;
                // ALUSrcB <= 2'b00;
                // PCWrite <= 1'b0;
                // PCSrc <= 1'b1;
                // R1Src <= 2'b00;
                // R2Src <= 1'b0;
                // SESF <= 1'b0;
                // PCWriteCond <= 1'b1;
                // MemRd <= 1'b0;
                // MemWr <= 1'b0;
                // MemToReg <= 1'b0;
                // ALU <= 3'b001;
                // RegWr <= 1'b0;
                // RegDst <= 1'b0;
                // BNEq <= 1'b1;
                // JE <= 1'b0;
                ALUSrcA <= 1'b1;
                ALUSrcB <= 2'b00;
                ALU <= 3'b001;
                PCSrc <= 1'b1;
                PCWriteCond = 1'b1;
                BNEq = 1'b1;
                RegDst = 1'b0;
            end
        5'd9: begin
                // IRd <= 1'b1;
                // ALUSrcA <= 1'b0;
                // ALUSrcB <= 2'b01;
                // PCWrite <= 1'b1;
                // PCSrc <= 1'b0;
                // R1Src <= 2'b00;
                // R2Src <= 1'b0;
                // SESF <= 1'b0;
                // PCWriteCond <= 1'b0;
                // MemRd <= 1'b0;
                // MemWr <= 1'b0;
                // MemToReg <= 1'b0;
                // ALU <= 3'b000;
                // RegWr <= 1'b0;
                // RegDst <= 1'b0;
                ALUSrcA <= 1'b1;
                ALUSrcB <= 2'b00;
                ALU <= 3'b001;
                PCSrc <= 1'b1;
                PCWriteCond = 1'b1;
                BNEq = 1'b0;
                RegDst = 1'b0;
            end
            5'd10: begin
            //    IRd <= 1'b1;
            //     ALUSrcA <= 1'b0;
            //     ALUSrcB <= 2'b01;
            //     PCWrite <= 1'b1;
            //     PCSrc <= 1'b0;
            //     R1Src <= 2'b00;
            //     R2Src <= 1'b0;
            //     SESF <= 1'b0;
            //     PCWriteCond <= 1'b0;
            //     MemRd <= 1'b0;
            //     MemWr <= 1'b0;
            //     MemToReg <= 1'b0;
            //     ALU <= 3'b000;
            //     RegWr <= 1'b0;
            //     RegDst <= 1'b0;
                ALUSrcA <= 1'b1;
                ALUSrcB <= 2'b10;
                SESF <= 1'b0;
                ALU <= 3'b000;
            end
            5'd11: begin
                // IRd <= 1'b1;
                // ALUSrcA <= 1'b0;
                // ALUSrcB <= 2'b01;
                // PCWrite <= 1'b1;
                // PCSrc <= 1'b0;
                // R1Src <= 2'b00;
                // R2Src <= 1'b0;
                // SESF <= 1'b0;
                // PCWriteCond <= 1'b0;
                // MemRd <= 1'b0;
                // MemWr <= 1'b0;
                // MemToReg <= 1'b0;
                // ALU <= 3'b000;
                // RegWr <= 1'b0;
                // RegDst <= 1'b0;
                ALUSrcA <= 1'b1;
                ALUSrcB <= 2'b10;
                SESF <= 1'b1;
                ALU <= 3'b000;
            end
            5'd12: begin
                // IRd <= 1'b1;
                // ALUSrcA <= 1'b0;
                // ALUSrcB <= 2'b01;
                // PCWrite <= 1'b1;
                // PCSrc <= 1'b0;
                // R1Src <= 2'b00;
                // R2Src <= 1'b0;
                // SESF <= 1'b0;
                // PCWriteCond <= 1'b0;
                // MemRd <= 1'b0;
                // MemWr <= 1'b0;
                // MemToReg <= 1'b0;
                // ALU <= 3'b000;
                // RegWr <= 1'b0;
                // RegDst <= 1'b0;
                ALUSrcA <= 1'b1;
                ALUSrcB <= 2'b10;
                SESF <= 1'b0;
                ALU <= 3'b001;
            end
            5'd13: begin
                // IRd <= 1'b1;
                // ALUSrcA <= 1'b0;
                // ALUSrcB <= 2'b01;
                // PCWrite <= 1'b1;
                // PCSrc <= 1'b0;
                // R1Src <= 2'b00;
                // R2Src <= 1'b0;
                // SESF <= 1'b0;
                // PCWriteCond <= 1'b0;
                // MemRd <= 1'b0;
                // MemWr <= 1'b0;
                // MemToReg <= 1'b0;
                // ALU <= 3'b000;
                // RegWr <= 1'b0;
                // RegDst <= 1'b0;
                ALUSrcA <= 1'b1;
                ALUSrcB <= 2'b10;
                SESF <= 1'b1;
                ALU <= 3'b001;
            end
            5'd14: begin
                // IRd <= 1'b1;
                // ALUSrcA <= 1'b0;
                // ALUSrcB <= 2'b01;
                // PCWrite <= 1'b1;
                // PCSrc <= 1'b0;
                // R1Src <= 2'b00;
                // R2Src <= 1'b0;
                // SESF <= 1'b0;
                // PCWriteCond <= 1'b0;
                // MemRd <= 1'b0;
                // MemWr <= 1'b0;
                // MemToReg <= 1'b0;
                // ALU <= 3'b000;
                // RegWr <= 1'b0;
                // RegDst <= 1'b0;
                ALUSrcA <= 1'b1;
                ALUSrcB <= 2'b10;
                SESF <= 1'b1;
                ALU <= 3'b110;
            end
            5'd15: begin
                // IRd <= 1'b1;
                // ALUSrcA <= 1'b0;
                // ALUSrcB <= 2'b01;
                // PCWrite <= 1'b1;
                // PCSrc <= 1'b0;
                // R1Src <= 2'b00;
                // R2Src <= 1'b0;
                // SESF <= 1'b0;
                // PCWriteCond <= 1'b0;
                // MemRd <= 1'b0;
                // MemWr <= 1'b0;
                // MemToReg <= 1'b0;
                // ALU <= 3'b000;
                // RegWr <= 1'b0;
                // RegDst <= 1'b0;
                ALUSrcA <= 1'b1;
                ALUSrcB <= 2'b10;
                SESF <= 1'b1;
                ALU <= 3'b101;
            end
            5'd16: begin
                // IRd <= 1'b1;
                // ALUSrcA <= 1'b0;
                // ALUSrcB <= 2'b01;
                // PCWrite <= 1'b1;
                // PCSrc <= 1'b0;
                // R1Src <= 2'b00;
                // R2Src <= 1'b0;
                // SESF <= 1'b0;
                // PCWriteCond <= 1'b0;
                // MemRd <= 1'b0;
                // MemWr <= 1'b0;
                // MemToReg <= 1'b0;
                // ALU <= 3'b000;
                // RegWr <= 1'b0;
                // RegDst <= 1'b0;
                ALUSrcA <= 1'b1;
                ALUSrcB <= 2'b10;
                SESF <= 1'b0;
                ALU <= 3'b010;
            end
            5'd17: begin
                // IRd <= 1'b1;
                // ALUSrcA <= 1'b0;
                // ALUSrcB <= 2'b01;
                // PCWrite <= 1'b1;
                // PCSrc <= 1'b0;
                // R1Src <= 2'b00;
                // R2Src <= 1'b0;
                // SESF <= 1'b0;
                // PCWriteCond <= 1'b0;
                // MemRd <= 1'b0;
                // MemWr <= 1'b0;
                // MemToReg <= 1'b0;
                // ALU <= 3'b000;
                // RegWr <= 1'b0;
                // RegDst <= 1'b0;
                ALUSrcA <= 1'b1;
                ALUSrcB <= 2'b10;
                SESF <= 1'b0;
                ALU <= 3'b011;
            end
            5'd18: begin
                // IRd <= 1'b1;
                // ALUSrcA <= 1'b0;
                // ALUSrcB <= 2'b01;
                // PCWrite <= 1'b1;
                // PCSrc <= 1'b0;
                // R1Src <= 2'b00;
                // R2Src <= 1'b0;
                // SESF <= 1'b0;
                // PCWriteCond <= 1'b0;
                // MemRd <= 1'b0;
                // MemWr <= 1'b0;
                // MemToReg <= 1'b0;
                // ALU <= 3'b000;
                // RegWr <= 1'b0;
                // RegDst <= 1'b0;
                ALUSrcA <= 1'b1;
                ALUSrcB <= 2'b10;
                SESF <= 1'b0;
                ALU <= 3'b100;
            end
            5'd19: begin
                IRd <= 1'b1;
                ALUSrcA <= 1'b0;
                ALUSrcB <= 2'b01;
                PCWrite <= 1'b1;
                PCSrc <= 1'b0;
                R1Src <= 2'b00;
                R2Src <= 1'b0;
                SESF <= 1'b0;
                PCWriteCond <= 1'b0;
                MemRd <= 1'b0;
                MemWr <= 1'b0;
                MemToReg <= 1'b0;
                ALU <= 3'b000;
                RegWr <= 1'b0;
                RegDst <= 1'b0;
            end
            5'd20: begin
                // IRd <= 1'b1;
                // ALUSrcA <= 1'b0;
                // ALUSrcB <= 2'b01;
                // PCWrite <= 1'b1;
                // PCSrc <= 1'b0;
                // R1Src <= 2'b00;
                // R2Src <= 1'b0;
                // SESF <= 1'b0;
                // PCWriteCond <= 1'b0;
                // MemRd <= 1'b0;
                // MemWr <= 1'b0;
                // MemToReg <= 1'b0;
                // ALU <= 3'b000;
                // RegWr <= 1'b0;
                // RegDst <= 1'b0;
                ALUSrcA <= 1'b0;
                SESF <= 1'b1;
                ALUSrcB <= 2'b10;
                ALU <= 3'b000;
                PCSrc <= 1'b0;
                PCWrite = 1'b1;
            end
            5'd21: begin
                // IRd <= 1'b1;
                // ALUSrcA <= 1'b0;
                // ALUSrcB <= 2'b01;
                // PCWrite <= 1'b1;
                // PCSrc <= 1'b0;
                // R1Src <= 2'b00;
                // R2Src <= 1'b0;
                // SESF <= 1'b0;
                // PCWriteCond <= 1'b0;
                // MemRd <= 1'b0;
                // MemWr <= 1'b0;
                // MemToReg <= 1'b0;
                // ALU <= 3'b000;
                // RegWr <= 1'b0;
                // RegDst <= 1'b0;
                MemToReg <= 1'b0;
                ALU <= 3'b001;
                PCWrite <= 1'b0;
                PCWriteCond <= 1'b0;
                RegDst <= 1'b0;
            end
            5'd22: begin
                IRd <= 1'b1;
                ALUSrcA <= 1'b0;
                ALUSrcB <= 2'b01;
                PCWrite <= 1'b1;
                PCSrc <= 1'b0;
                R1Src <= 2'b00;
                R2Src <= 1'b0;
                SESF <= 1'b0;
                PCWriteCond <= 1'b0;
                MemRd <= 1'b0;
                MemWr <= 1'b0;
                MemToReg <= 1'b0;
                ALU <= 3'b000;
                RegWr <= 1'b0;
                RegDst <= 1'b0;
            end
            5'd23: begin
                IRd <= 1'b1;
                ALUSrcA <= 1'b0;
                ALUSrcB <= 2'b01;
                PCWrite <= 1'b1;
                PCSrc <= 1'b0;
                R1Src <= 2'b00;
                R2Src <= 1'b0;
                SESF <= 1'b0;
                PCWriteCond <= 1'b0;
                MemRd <= 1'b0;
                MemWr <= 1'b0;
                MemToReg <= 1'b0;
                ALU <= 3'b000;
                RegWr <= 1'b0;
                RegDst <= 1'b0;
            end
            5'd24: begin
                IRd <= 1'b1;
                ALUSrcA <= 1'b0;
                ALUSrcB <= 2'b01;
                PCWrite <= 1'b1;
                PCSrc <= 1'b0;
                R1Src <= 2'b00;
                R2Src <= 1'b0;
                SESF <= 1'b0;
                PCWriteCond <= 1'b0;
                MemRd <= 1'b0;
                MemWr <= 1'b0;
                MemToReg <= 1'b0;
                ALU <= 3'b000;
                RegWr <= 1'b0;
                RegDst <= 1'b0;
            end

        endcase
    end

endmodule

module MIPS(clk, rst);
    input clk, rst;

    wire IRd, ALUSrcA, PCWrite, PCSrc, R2Src, SESF, PCWriteCond, MemRd, MemWr, MemToReg, RegWr, RegDst, JE, BNEq;

    wire [1:0] ALUSrcB,R1Src;
    wire reg [2:0] ALUCtrl;

    wire [3:0] OpCode;
    wire [3:0] Func;

    Datapath Datapath ( .clk(clk)
                        .rst(rst),
                        .PCWrite(PCWrite),
                        .PCWriteCond(PCWriteCond),
                        .BNEq(BNEq),
                        .MemRd(MemRd),
                        .MemWr(MemWr),
                        .IRd(IRd),
                        .IRWr(IRWr),
                        .RegWrite(RegWrite),
                        .PCWrite_in(PCWrite_in),
                        .RegDst(RegDst),
                        .MemToReg(MemToReg),
                        .SESF(SESF),
                        .JE(JE),
                        .ALUSrcA(ALUSrcA),
                        .R1Src(R1Src),
                        .ALUSrcB(ALUSrcB),
                        .PCSrc(PCSrc),
                        .ALUCtrl(ALUCtrl),
                        .OpCode(OpCode),
                        .Func(Func)
                        );



    Control ControlUnit ( .clk(clk)
                        .rst(rst),
                        .PCWrite(PCWrite),
                        .PCWriteCond(PCWriteCond),
                        .BNEq(BNEq),
                        .MemRd(MemRd),
                        .MemWr(MemWr),
                        .IRd(IRd),
                        .IRWr(IRWr),
                        .RegWrite(RegWrite),
                        .PCWrite_in(PCWrite_in),
                        .RegDst(RegDst),
                        .MemToReg(MemToReg),
                        .SESF(SESF),
                        .JE(JE),
                        .ALUSrcA(ALUSrcA),
                        .R1Src(R1Src),
                        .ALUSrcB(ALUSrcB),
                        .PCSrc(PCSrc),
                        .ALU(ALUCtrl),
                        .OpCode(OpCode),
                        .Func(Func));

endmodule



