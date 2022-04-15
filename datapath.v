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

// module tb_alu();
// 	reg [15:0]A,B;
// 	reg [2:0] operation;

// 	wire [15:0]out;
// 	wire zero;

// 	alu uut(.ALU_Out(out), .ZF(zero), .A(A), .B(B), .ALU_Sel(operation));
// 	initial 
// 	begin
//         A = 16'b1000000000001000; B = 16'd3; operation = 3'd0; //1000000000000001
//         #10 A = 16'b1000000000001000; B = 16'd3; operation = 3'd1;
//         #10 A = 16'b1000000000001000; B = 16'b1000000000001000; operation = 3'd1;
//         #10 A = 16'b1000000000001000; B = 16'd3; operation = 3'd2;
//         #10 A = 16'b1000000000001000; B = 16'd3; operation = 3'd3;
//         #10 A = 16'b1000000000001000; B = 16'd3; operation = 3'd4;
//         #10 A = 16'b1000000000001000; B = 16'd3; operation = 3'd5;
//         #10 A = 16'b1000000000001000; B = 16'd3; operation = 3'd6;
//         #10 A = 16'b1000000000001000; B = 16'd3; operation = 3'd7;
        
// 	end
// 	initial begin
//         $monitor("A=%b, B=%b, Output=%b, zero=%b",A,B,out,zero);
//     end
// 	initial begin
// 		#300 $finish;
// 	end
// 	endmodule

module zero_padder(
    input 
)
endmodule

module instr_mem(
    input[15:0] pc,
    output[15:0] instruction
);

    reg [16:0] memory [32767:0];
    wire [14 : 0] address = pc[15 : 1];

    assign instruction =  memory[address];

endmodule

module data_mem(
    input[15:0] pc,
    output[15:0] data
);

    reg [16:0] memory [32767:0];
    wire [14 : 0] address = pc[15 : 1];

    assign data =  memory[address];

endmodule



