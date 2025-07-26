// Copyright (C) 2017  Intel Corporation. All rights reserved.
// Your use of Intel Corporation's design tools, logic functions 
// and other software and tools, and its AMPP partner logic 
// functions, and any output files from any of the foregoing 
// (including device programming or simulation files), and any 
// associated documentation or information are expressly subject 
// to the terms and conditions of the Intel Program License 
// Subscription Agreement, the Intel Quartus Prime License Agreement,
// the Intel FPGA IP License Agreement, or other applicable license
// agreement, including, without limitation, that your use is for
// the sole purpose of programming logic devices manufactured by
// Intel and sold by Intel or its authorized distributors.  Please
// refer to the applicable agreement for further details.

// PROGRAM		"Quartus Prime"
// VERSION		"Version 17.1.0 Build 590 10/25/2017 SJ Lite Edition"
// CREATED		"Sat Jul 26 14:54:51 2025"

module Processor(
	clk_fpga,
	Enter,
	Reset,
	IR_IN,
	LCD_ON,
	LCD_BLON,
	LCD_RW,
	LCD_EN,
	LCD_RS,
	currproc,
	display_Hundreds,
	display_Ones,
	display_Tens,
	GPIO_OU,
	hex3,
	hex4,
	hex5,
	hex6,
	hex7,
	ilcd,
	LCD_DATA,
	O_PC,
	opcode,
	pmpc,
	prevproc
);


input wire	clk_fpga;
input wire	Enter;
input wire	Reset;
input wire	IR_IN;
output wire	LCD_ON;
output wire	LCD_BLON;
output wire	LCD_RW;
output wire	LCD_EN;
output wire	LCD_RS;
output wire	[3:0] currproc;
output wire	[6:0] display_Hundreds;
output wire	[6:0] display_Ones;
output wire	[6:0] display_Tens;
output wire	[7:0] GPIO_OU;
output wire	[6:0] hex3;
output wire	[6:0] hex4;
output wire	[6:0] hex5;
output wire	[6:0] hex6;
output wire	[6:0] hex7;
output wire	[15:0] ilcd;
inout wire	[7:0] LCD_DATA;
output wire	[31:0] O_PC;
output wire	[5:0] opcode;
output wire	[31:0] pmpc;
output wire	[3:0] prevproc;

wire	[31:0] adderResult;
wire	[3:0] ALU_control;
wire	[31:0] ALU_result;
wire	ALUSrc;
wire	[31:0] binary;
wire	BNE;
wire	Branch;
wire	BranchOp;
wire	CLOCK;
wire	CLOCK50;
wire	CLOCK_R;
wire	[6:0] do;
wire	dOut;
wire	[31:0] extended;
wire	[3:0] Hundreds;
wire	inData;
wire	[16:0] inputData;
wire	[15:0] inputLCD;
wire	[31:0] instruction;
wire	[31:0] jAddress;
wire	[1:0] Jump;
wire	[1:0] MemToReg;
wire	MemWrite;
wire	[31:0] mtrData;
wire	[31:0] MUX_ALUSrc_result;
wire	[31:0] MUX_BZ_result;
wire	[31:0] MUX_Jump_result;
wire	nIn;
wire	[3:0] Ones;
wire	outData;
wire	[31:0] PC;
wire	[31:0] PCadderResult;
wire	PCwrite;
wire	[31:0] pmPC_ALTERA_SYNTHESIZED;
wire	proc10;
wire	proc100;
wire	proc101;
wire	proc1010;
wire	proc1011;
wire	proc1012;
wire	proc1013;
wire	proc1014;
wire	proc1015;
wire	proc1016;
wire	proc1017;
wire	proc1018;
wire	proc1019;
wire	proc102;
wire	proc1020;
wire	proc1021;
wire	proc1022;
wire	proc1023;
wire	proc1024;
wire	proc1025;
wire	proc1026;
wire	proc1027;
wire	proc1028;
wire	proc1029;
wire	proc103;
wire	proc1030;
wire	proc1031;
wire	proc104;
wire	proc105;
wire	proc106;
wire	proc107;
wire	proc108;
wire	proc109;
wire	proc11;
wire	proc110;
wire	proc111;
wire	proc112;
wire	proc113;
wire	proc114;
wire	proc115;
wire	proc116;
wire	proc117;
wire	proc118;
wire	proc119;
wire	proc12;
wire	proc120;
wire	proc121;
wire	proc122;
wire	proc123;
wire	proc124;
wire	proc125;
wire	proc126;
wire	proc127;
wire	proc128;
wire	proc129;
wire	proc13;
wire	proc130;
wire	proc131;
wire	proc14;
wire	proc15;
wire	proc16;
wire	proc17;
wire	proc18;
wire	proc19;
wire	[31:0] proc2;
wire	[31:0] proc3;
wire	[31:0] proc4;
wire	[31:0] proc5;
wire	[31:0] proc6;
wire	[31:0] proc7;
wire	[31:0] proc8;
wire	[31:0] proc9;
wire	[31:0] rd;
wire	[31:0] read_Data;
wire	[31:0] readData1;
wire	[31:0] readData2;
wire	[1:0] RegDst;
wire	RegWrite;
wire	[3:0] Tens;
wire	[3:0] Thousands;
wire	[31:0] writeData;
wire	[4:0] writeRegister;
wire	Zero;
wire	SYNTHESIZED_WIRE_0;
wire	SYNTHESIZED_WIRE_1;
wire	SYNTHESIZED_WIRE_2;
wire	SYNTHESIZED_WIRE_3;





clock_divider	b2v_cd(
	.n_clock(CLOCK50),
	.clk(CLOCK),
	.clk_read(CLOCK_R));


control_unit	b2v_CU(
	.Enter(Enter),
	.funct(instruction[5:0]),
	.opcode(instruction[31:26]),
	.Branch(Branch),
	.MemWrite(MemWrite),
	.ALUSrc(ALUSrc),
	.RegWrite(RegWrite),
	.BNE(BNE),
	.inData(inData),
	.outData(outData),
	.PCwrite(PCwrite),
	.ALUOp(ALU_control),
	.Jump(Jump),
	.MemToReg(MemToReg),
	.RegDst(RegDst));


data_memory	b2v_datamem(
	.we(MemWrite),
	.read_clock(CLOCK_R),
	.write_clock(CLOCK),
	.data(readData2),
	.read_addr(ALU_result[9:0]),
	.write_addr(ALU_result[9:0]),
	.proc131(proc131),
	.proc130(proc130),
	.proc129(proc129),
	.proc128(proc128),
	.proc127(proc127),
	.proc126(proc126),
	.proc125(proc125),
	.proc124(proc124),
	.proc123(proc123),
	.proc122(proc122),
	.proc121(proc121),
	.proc120(proc120),
	.proc119(proc119),
	.proc118(proc118),
	.proc117(proc117),
	.proc116(proc116),
	.proc115(proc115),
	.proc114(proc114),
	.proc113(proc113),
	.proc112(proc112),
	.proc111(proc111),
	.proc110(proc110),
	.proc19(proc19),
	.proc18(proc18),
	.proc17(proc17),
	.proc16(proc16),
	.proc15(proc15),
	.proc14(proc14),
	.proc13(proc13),
	.proc12(proc12),
	.proc11(proc11),
	.proc10(proc10),
	.proc1031(proc1031),
	.proc1030(proc1030),
	.proc1029(proc1029),
	.proc1028(proc1028),
	.proc1027(proc1027),
	.proc1026(proc1026),
	.proc1025(proc1025),
	.proc1024(proc1024),
	.proc1023(proc1023),
	.proc1022(proc1022),
	.proc1021(proc1021),
	.proc1020(proc1020),
	.proc1019(proc1019),
	.proc1018(proc1018),
	.proc1017(proc1017),
	.proc1016(proc1016),
	.proc1015(proc1015),
	.proc1014(proc1014),
	.proc1013(proc1013),
	.proc1012(proc1012),
	.proc1011(proc1011),
	.proc1010(proc1010),
	.proc109(proc109),
	.proc108(proc108),
	.proc107(proc107),
	.proc106(proc106),
	.proc105(proc105),
	.proc104(proc104),
	.proc103(proc103),
	.proc102(proc102),
	.proc101(proc101),
	.proc100(proc100),
	.proc2(proc2),
	.proc3(proc3),
	.proc4(proc4),
	.proc5(proc5),
	.proc6(proc6),
	.proc7(proc7),
	.proc8(proc8),
	.proc9(proc9),
	.q(read_Data));
	defparam	b2v_datamem.ADDR_WIDTH = 10;
	defparam	b2v_datamem.DATA_WIDTH = 32;


debounce	b2v_deb(
	.pb_1(nIn),
	.clk(CLOCK),
	.pb_out(dOut));


BCD	b2v_decbcd(
	.binary(binary[12:0]),
	.Hundreds(Hundreds),
	.Ones(Ones),
	.Tens(Tens),
	.Thousands(Thousands));

assign	nIn =  ~Reset;


adder	b2v_GeneralAdder(
	.X(PCadderResult),
	.Y(extended),
	.result(adderResult));


decoder	b2v_hex0(
	.In(Ones),
	.Out(display_Ones));


decoder	b2v_hex1(
	.In(Tens),
	.Out(display_Tens));


decoder	b2v_hex2(
	.In(Hundreds),
	.Out(display_Hundreds));


decoder_off	b2v_hex3to7(
	.Out(do));


decoder	b2v_hex_3(
	.In(Thousands),
	.Out(hex3));


InputFPGA	b2v_inF(
	.iCLK(CLOCK50),
	.iRST_n(SYNTHESIZED_WIRE_0),
	.iIRDA(IR_IN),
	.Data(inputData));


muxInput	b2v_inputmux(
	.In(inData),
	.Out(outData),
	.inputFPGA(inputData),
	.inputMUX(mtrData),
	.outputFPGA(binary),
	.result(writeData));

assign	SYNTHESIZED_WIRE_3 = Zero & Branch;

assign	SYNTHESIZED_WIRE_2 = BNE & SYNTHESIZED_WIRE_1;

assign	SYNTHESIZED_WIRE_1 =  ~Zero;

assign	BranchOp = SYNTHESIZED_WIRE_2 | SYNTHESIZED_WIRE_3;


instruction_memory	b2v_instmem(
	.clk(CLOCK),
	.addr(PC[11:0]),
	.q(instruction));
	defparam	b2v_instmem.ADDR_WIDTH = 12;
	defparam	b2v_instmem.DATA_WIDTH = 32;


jump_aux	b2v_jumpaux(
	.in26b(instruction[25:0]),
	.jumpAddress(jAddress));


LCD	b2v_LCD(
	.CLOCK_50(CLOCK50),
	.clk(CLOCK),
	.inputSignal(inputLCD),
	.LCD_DATA(LCD_DATA),
	.LCD_ON(LCD_ON),
	.LCD_BLON(LCD_BLON),
	.LCD_RW(LCD_RW),
	.LCD_EN(LCD_EN),
	.LCD_RS(LCD_RS)
	);


mux32b	b2v_muxALUSrc(
	.control(ALUSrc),
	.in1(readData2),
	.in2(extended),
	.result(MUX_ALUSrc_result));


mux32b	b2v_muxBranchOp(
	.control(BranchOp),
	.in1(PCadderResult),
	.in2(adderResult),
	.result(MUX_BZ_result));


mux32b_v2	b2v_muxJump(
	.control(Jump),
	.in1(MUX_BZ_result),
	.in2(jAddress),
	.in3(readData1),
	.result(MUX_Jump_result));


mux32b_v2	b2v_muxmtr(
	.control(MemToReg),
	.in1(ALU_result),
	.in2(read_Data),
	.in3(PCadderResult),
	.result(mtrData));


mux5b	b2v_muxregdst(
	.rd(instruction[15:11]),
	.regDst(RegDst),
	.rt(instruction[20:16]),
	.rdst(writeRegister));

assign	SYNTHESIZED_WIRE_0 =  ~dOut;


PC_adder	b2v_PCadder(
	.X(PC),
	.result(PCadderResult));


PC	b2v_pgmc(
	.reset(dOut),
	.PCwrite(PCwrite),
	.nextPC(pmPC_ALTERA_SYNTHESIZED),
	.program_counter(PC));


procmanager	b2v_pm(
	.clk(CLOCK),
	.reset(dOut),
	.PCwrite(PCwrite),
	.nextPC(MUX_Jump_result),
	.opcode(instruction[31:26]),
	.proc131(proc131),
	.proc130(proc130),
	.proc129(proc129),
	.proc128(proc128),
	.proc127(proc127),
	.proc126(proc126),
	.proc125(proc125),
	.proc124(proc124),
	.proc123(proc123),
	.proc122(proc122),
	.proc121(proc121),
	.proc120(proc120),
	.proc119(proc119),
	.proc118(proc118),
	.proc117(proc117),
	.proc116(proc116),
	.proc115(proc115),
	.proc114(proc114),
	.proc113(proc113),
	.proc112(proc112),
	.proc111(proc111),
	.proc110(proc110),
	.proc19(proc19),
	.proc18(proc18),
	.proc17(proc17),
	.proc16(proc16),
	.proc15(proc15),
	.proc14(proc14),
	.proc13(proc13),
	.proc12(proc12),
	.proc11(proc11),
	.proc10(proc10),
	.proc1031(proc1031),
	.proc1030(proc1030),
	.proc1029(proc1029),
	.proc1028(proc1028),
	.proc1027(proc1027),
	.proc1026(proc1026),
	.proc1025(proc1025),
	.proc1024(proc1024),
	.proc1023(proc1023),
	.proc1022(proc1022),
	.proc1021(proc1021),
	.proc1020(proc1020),
	.proc1019(proc1019),
	.proc1018(proc1018),
	.proc1017(proc1017),
	.proc1016(proc1016),
	.proc1015(proc1015),
	.proc1014(proc1014),
	.proc1013(proc1013),
	.proc1012(proc1012),
	.proc1011(proc1011),
	.proc1010(proc1010),
	.proc109(proc109),
	.proc108(proc108),
	.proc107(proc107),
	.proc106(proc106),
	.proc105(proc105),
	.proc104(proc104),
	.proc103(proc103),
	.proc102(proc102),
	.proc101(proc101),
	.proc100(proc100),
	.proc2(proc2),
	.proc3(proc3),
	.proc4(proc4),
	.proc5(proc5),
	.proc6(proc6),
	.proc7(proc7),
	.proc8(proc8),
	.proc9(proc9),
	
	
	
	
	
	
	.inputLCD(inputLCD),
	.newPC(pmPC_ALTERA_SYNTHESIZED)
	
	);


register_file	b2v_regfile(
	.regWrite(RegWrite),
	.clk(CLOCK),
	.readRegister1(instruction[25:21]),
	.readRegister2(instruction[20:16]),
	.writeData(writeData),
	.writeRegister(writeRegister),
	
	.readData1(readData1),
	.readData2(readData2));


sign_extend	b2v_Sign_Extend(
	.extend(instruction[15:0]),
	.extended(extended));


ALU	b2v_ula(
	.ALU_control(ALU_control),
	.X(readData1),
	.Y(MUX_ALUSrc_result),
	.Zero(Zero),
	.result(ALU_result));

assign	CLOCK50 = clk_fpga;
assign	GPIO_OU[7:0] = inputData[7:0];
assign	hex4 = do;
assign	hex5 = do;
assign	hex6 = do;
assign	hex7 = do;
assign	ilcd = inputLCD;
assign	O_PC = PC;
assign	opcode[5:0] = instruction[31:26];
assign	pmpc = pmPC_ALTERA_SYNTHESIZED;

endmodule
