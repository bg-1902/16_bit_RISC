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