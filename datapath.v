module alu(
    input [15:0] A,B,                  
    input [2:0] ALU_Sel,
    output [15:0] ALU_Out,
    output reg ZF
    );

    reg [15:0] ALU_Result;
    assign ALU_Out = ALU_Result; // ALU out

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

module tb_alu();
	reg [15:0]A,B;
	reg [2:0] operation;

	wire [15:0]out;
	wire zero;

	alu uut(.ALU_Out(out), .ZF(zero), .A(A), .B(B), .ALU_Sel(operation));
	initial 
	begin
        A = 16'b1000000000001000; B = 16'd3; operation = 3'd0; //1000000000000001
        #10 A = 16'b1000000000001000; B = 16'd3; operation = 3'd1;
        #10 A = 16'b1000000000001000; B = 16'b1000000000001000; operation = 3'd1;
        #10 A = 16'b1000000000001000; B = 16'd3; operation = 3'd2;
        #10 A = 16'b1000000000001000; B = 16'd3; operation = 3'd3;
        #10 A = 16'b1000000000001000; B = 16'd3; operation = 3'd4;
        #10 A = 16'b1000000000001000; B = 16'd3; operation = 3'd5;
        #10 A = 16'b1000000000001000; B = 16'd3; operation = 3'd6;
        #10 A = 16'b1000000000001000; B = 16'd3; operation = 3'd7;
        
	end
	initial begin
        $monitor("A=%b, B=%b, Output=%b, zero=%b",A,B,out,zero);
    end
	initial begin
		#300 $finish;
	end
	endmodule

// module zero_padder(
//     input 
// )
// endmodule

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

    always @(posedge clk) begin
        if(IRd == 1'b1) begin
            instruction <= memory[address];
        end
    end

endmodule

module data_mem(
    input clk;
    input[15:0] pc,
    input [15:0] WrData;
    input MemRd, MemWr;
    output[15:0] data
);

    reg [15:0] memory [0:32767];
    wire [14 : 0] address = pc[15 : 1];

    always @(posedge clk) begin
        if(MemRd == 1'b1) begin
            data <= memory[address];
        end
    end

    always @(posedge clk) begin
        if(MemWr == 1'b1) begin
            memory[address] <= WrData;
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
        Registers[0] <= 16'd0;
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

module Datapath();
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
    input wire RegDest;
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

    //Internal Wires
    





endmodule








