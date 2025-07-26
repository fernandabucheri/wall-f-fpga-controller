module control_unit(opcode, funct, Enter, RegDst, Jump, Branch, MemToReg, ALUOp, MemWrite, ALUSrc, RegWrite, BNE, inData, outData, PCwrite);

input Enter;
input wire [5:0] opcode, funct;
output reg Branch, MemWrite, ALUSrc, RegWrite, BNE, inData, outData, PCwrite;
output reg [3:0] ALUOp;
output reg [1:0] MemToReg, RegDst, Jump;

always@(*)
begin
	if(opcode == 6'b000000)
	begin
		RegDst = 2'b01;
		Branch = 1'b0;
		MemToReg = 2'b00;
		MemWrite = 1'b0;
		ALUSrc = 1'b0;
		BNE = 1'b0;
		outData = 1'b0;
		inData = 1'b0;
		PCwrite = 1'b1;
		case(funct) //ALU_Control
			6'b000001: begin ALUOp = 4'b0000; Jump = 2'b00; RegWrite = 1'b1; end //add
			6'b000010: begin ALUOp = 4'b0001; Jump = 2'b00; RegWrite = 1'b1; end //sub
			6'b000011: begin ALUOp = 4'b0010; Jump = 2'b00; RegWrite = 1'b1; end //mult
			6'b000100: begin ALUOp = 4'b0011; Jump = 2'b00; RegWrite = 1'b1; end //div
			6'b000101: begin ALUOp = 4'b0100; Jump = 2'b00; RegWrite = 1'b1; end //and
			6'b000110: begin ALUOp = 4'b0101; Jump = 2'b00; RegWrite = 1'b1; end //or
			6'b000111: begin ALUOp = 4'b0110; Jump = 2'b00; RegWrite = 1'b1; end //nor
			6'b001000: begin ALUOp = 4'b1111; Jump = 2'b10; RegWrite = 1'b0; end //jr
			default: begin ALUOp = 4'b1111; Jump = 2'b00; RegWrite = 1'b1; end
		endcase
	end
	else if(opcode == 6'b000001) //addi
	begin
		RegDst = 2'b00;
		Jump = 2'b00;
		Branch = 1'b0;
		MemToReg = 2'b00;
		MemWrite = 1'b0;
		ALUSrc = 1'b1;
		RegWrite = 1'b1;
		BNE = 1'b0;
		outData = 1'b0;
		inData = 1'b0;
		PCwrite = 1'b1;
		ALUOp = 4'b0000;
	end
	else if(opcode == 6'b000010) //subi
	begin
		RegDst = 2'b00;
		Jump = 2'b00;
		Branch = 1'b0;
		MemToReg = 2'b00;
		MemWrite = 1'b0;
		ALUSrc = 1'b1;
		RegWrite = 1'b1;
		BNE = 1'b0;
		outData = 1'b0;
		inData = 1'b0;
		PCwrite = 1'b1;
		ALUOp = 4'b0001;
	end
	else if(opcode == 6'b000011) //multi
	begin
		RegDst = 2'b00;
		Jump = 2'b00;
		Branch = 1'b0;
		MemToReg = 2'b00;
		MemWrite = 1'b0;
		ALUSrc = 1'b1;
		RegWrite = 1'b1;
		BNE = 1'b0;
		outData = 1'b0;
		inData = 1'b0;
		PCwrite = 1'b1;
		ALUOp = 4'b0010;
	end
	else if(opcode == 6'b000100) //divi
	begin
		RegDst = 2'b00;
		Jump = 2'b00;
		Branch = 1'b0;
		MemToReg = 2'b00;
		MemWrite = 1'b0;
		ALUSrc = 1'b1;
		RegWrite = 1'b1;
		BNE = 1'b0;
		outData = 1'b0;
		inData = 1'b0;
		PCwrite = 1'b1;
		ALUOp = 4'b0011;
	end
	else if(opcode == 6'b000101) //input
	begin
		RegDst = 2'b01;
		Jump = 2'b00;
		Branch = 1'b0;
		MemToReg = 2'b00;
		MemWrite = 1'b0;
		ALUSrc = 1'b0;
		RegWrite = 1'b1;
		BNE = 1'b0;
		outData = 1'b1;
		inData = 1'b1;
		PCwrite = 1'b1;
// 		PCwrite = Enter? 1'b1 : 1'b0;
		ALUOp = 4'b1111;
	end
	else if(opcode == 6'b000110) //output
	begin
		RegDst = 2'b00;
		Jump = 2'b00;
		Branch = 1'b0;
		MemToReg = 2'b00;
		MemWrite = 1'b0;
		ALUSrc = 1'b1;
		RegWrite = 1'b1;
		BNE = 1'b0;
		outData = 1'b1;
		inData = 1'b0;
//		PCwrite = 1'b1;
		PCwrite = Enter? 1'b1 : 1'b0;
		ALUOp = 4'b0000;
	end
	else if(opcode == 6'b000111) //lw
	begin
		RegDst = 2'b00;
		Jump = 2'b00;
		Branch = 1'b0;
		MemToReg = 2'b01;
		MemWrite = 1'b0;
		ALUSrc = 1'b1;
		RegWrite = 1'b1;
		BNE = 1'b0;
		outData = 1'b0;
		inData = 1'b0;
		PCwrite = 1'b1;
		ALUOp = 4'b0000;
	end
	else if(opcode == 6'b001000) //sw
	begin
		RegDst = 2'b00;
		Jump = 2'b00;
		Branch = 1'b0;
		MemToReg = 2'b00;
		MemWrite = 1'b1;
		ALUSrc = 1'b1;
		RegWrite = 1'b0;
		BNE = 1'b0;
		outData = 1'b0;
		inData = 1'b0;
		PCwrite = 1'b1;
		ALUOp = 4'b0000;
	end
	else if(opcode == 6'b001001) //beq
	begin
		RegDst = 2'b00;
		Jump = 2'b00;
		Branch = 1'b1;
		MemToReg = 2'b00;
		MemWrite = 1'b0;
		ALUSrc = 1'b0;
		RegWrite = 1'b0;
		BNE = 1'b0;
		outData = 1'b0;
		inData = 1'b0;
		PCwrite = 1'b1;
		ALUOp = 4'b0001;
	end
	else if(opcode == 6'b001010) //bne
	begin
		RegDst = 2'b00;
		Jump = 1'b00;
		Branch = 1'b0;
		MemToReg = 2'b00;
		MemWrite = 1'b0;
		ALUSrc = 1'b0;
		RegWrite = 1'b0;
		BNE = 1'b1;
		outData = 1'b0;
		inData = 1'b0;
		PCwrite = 1'b1;
		ALUOp = 4'b0001;
	end
	else if(opcode == 6'b001011) //slt
	begin
		RegDst = 2'b01;
		Jump = 2'b00;
		Branch = 1'b0;
		MemToReg = 2'b00;
		MemWrite = 1'b0;
		ALUSrc = 1'b0;
		RegWrite = 1'b1;
		BNE = 1'b0;
		outData = 1'b0;
		inData = 1'b0;
		PCwrite = 1'b1;
		ALUOp = 4'b0111;
	end
	else if(opcode == 6'b001100) //slti
	begin
		RegDst = 2'b00;
		Jump = 2'b00;
		Branch = 1'b0;
		MemToReg = 2'b00;
		MemWrite = 1'b0;
		ALUSrc = 1'b1;
		RegWrite = 1'b1;
		BNE = 1'b0;
		outData = 1'b0;
		inData = 1'b0;
		PCwrite = 1'b1;
		ALUOp = 4'b0111;
	end
	else if(opcode == 6'b001101) //j
	begin
		RegDst = 2'b00;
		Jump = 2'b01;
		Branch = 1'b0;
		MemToReg = 2'b00;
		MemWrite = 1'b0;
		ALUSrc = 1'b0;
		RegWrite = 1'b0;
		BNE = 1'b0;
		outData = 1'b0;
		inData = 1'b0;
		PCwrite = 1'b1;
		ALUOp = 4'b1111;
	end
	else if(opcode == 6'b001110) //jal
	begin
		RegDst = 2'b10;
		Jump = 2'b01;
		Branch = 1'b0;
		MemToReg = 2'b10;
		MemWrite = 1'b0;
		ALUSrc = 1'b0;
		RegWrite = 1'b1;
		BNE = 1'b0;
		outData = 1'b0;
		inData = 1'b0;
		PCwrite = 1'b1;
		ALUOp = 4'b1111;
	end
	else if(opcode == 6'b010000) //endmain
	begin
		RegDst = 2'b00;
		Jump = 2'b00;
		Branch = 1'b0;
		MemToReg = 2'b00;
		MemWrite = 1'b0;
		ALUSrc = 1'b0;
		RegWrite = 1'b0;
		BNE = 1'b0;
		outData = 1'b0;
		inData = 1'b0;
		PCwrite = 1'b0;
		ALUOp = 4'b1111;
	end
	else if(opcode == 6'b111101) // endProc
	begin
		RegDst = 2'b00;
		Jump = 2'b00;
		Branch = 1'b0;
		MemToReg = 2'b00;
		MemWrite = 1'b0;
		ALUSrc = 1'b0;
		RegWrite = 1'b0;
		BNE = 1'b0;
		outData = 1'b1;
		inData = 1'b0;
//		PCwrite = 1'b1;
		PCwrite = Enter? 1'b1 : 1'b0;
		ALUOp = 4'b1111;
	end
	else if(opcode == 6'b111110) // procmanager
	begin
		RegDst = 2'b00;
		Jump = 2'b00;
		Branch = 1'b0;
		MemToReg = 2'b00;
		MemWrite = 1'b0;
		ALUSrc = 1'b0;
		RegWrite = 1'b0;
		BNE = 1'b0;
		outData = 1'b1;
		inData = 1'b0;
		PCwrite = 1'b1;
		ALUOp = 4'b1111;
	end
	else 
	begin
		RegDst = 2'b00;
		Jump = 2'b00;
		Branch = 1'b0;
		MemToReg = 2'b00;
		MemWrite = 1'b0;
		ALUSrc = 1'b0;
		RegWrite = 1'b0;
		BNE = 1'b0;
		outData = 1'b0;
		inData = 1'b0;
		PCwrite = 1'b0;
		ALUOp = 4'b1111;
	end
end

endmodule