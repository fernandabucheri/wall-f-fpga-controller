// Copyright (C) 2020  Intel Corporation. All rights reserved.
// Your use of Intel Corporation's design tools, logic functions 
// and other software and tools, and any partner logic 
// functions, and any output files from any of the foregoing 
// (including device programming or simulation files), and any 
// associated documentation or information are expressly subject 
// to the terms and conditions of the Intel Program License 
// Subscription Agreement, the Intel Quartus Prime License Agreement,
// the Intel FPGA IP License Agreement, or other applicable license
// agreement, including, without limitation, that your use is for
// the sole purpose of programming logic devices manufactured by
// Intel and sold by Intel or its authorized distributors.  Please
// refer to the applicable agreement for further details, at
// https://fpgasoftware.intel.com/eula.

// PROGRAM		"Quartus Prime"
// VERSION		"Version 20.1.1 Build 720 11/11/2020 SJ Lite Edition"
// CREATED		"Sat Jul 23 13:29:10 2022"

module SistemaComputacional(
	GPIO_IN, 
	V_BT,
	HEX7,
	HEX6,
	HEX5,
	HEX4,
	HEX3,
	HEX2,
	HEX1,
	HEX0, 
	CLOCK_50,
	LCD_DATA, LCD_EN, LCD_RW, LCD_RS, LCD_ON, LCD_BLON,
	UART_TXD
);

output [7:0] LCD_DATA;
output LCD_EN;
output LCD_RW;
output LCD_RS;
output LCD_ON;
output LCD_BLON;
reg lcd_trd_msg = 1;

output UART_TXD;

input wire [17:0] GPIO_IN;
input wire [3:0] V_BT;
output [0:6] HEX7, HEX6, HEX5, HEX4, HEX3, HEX2, HEX1, HEX0;
input wire CLOCK_50;
wire[31:0] numeroInstrucao, Saida;
wire saidaclock1, saidaclock2;

wire	fioclockwrite;
wire 	fioclock;
wire	[31:0] fioSaidaMux4EntradaPC;
wire	[31:0] fioExtensor;
wire	[31:0] fioMemoriaDeDados;
wire	[31:0] fioSomadorPC;
wire	[31:0] fioResultadoULA;
wire	[1:0] fioSinalControleMemToReg;
wire	[31:0] fioSaidaPC;
wire	[31:0] fioInstrucao;
wire	fioSinalControleZero;
wire	fioSinalControleBeqOp;
wire	fioSinalControleBneqOp;
wire	[31:0] fioSaidaSomadorPC;
wire	[31:0] fioDado1BR;
wire	[31:0] fioSaidaExtensorDeSinal2;
wire	[31:0] fioSaidaMux3;
wire	[1:0] fioSinalControleJumpOp;
wire	fioSinalControleIn;
wire	fioSinalControleOut;
wire	[16:0] fioentradaFPGA;
wire	[31:0] fioSaidaMux5;
wire	fioSinalControleEscritaBDR;
wire	[31:0] fioDadoParaEscrita;
wire	[3:0] fioRegParaEscrita;
wire	[31:0] fioSaidaMux6DadoEscritaBR;
wire	[1:0] fioSinalControleRegDest;
wire	fioSinalControleSize;
wire	fioSinalControleUlaSrc;
wire	[31:0] fioDado2BR;
wire	[31:0] fioOperando2;
wire	[3:0] fioSinalControleUlaOp;
wire	fioSinalControleMemDadosEscrita;
wire	fioSinalControleMemDadosLeitura;
wire	[3:0] fioEntradaA;
wire	[3:0] fioEntradaB;
wire	[3:0] fioEntradaC;
wire	[3:0] fioEntradaD;
wire	[3:0] fioEntradaE;
wire	[3:0] fioEntradaF;
wire	[3:0] fioEntradaG;
wire	[3:0] fioEntradaH;
wire  fioSemPreemp,  fioEncerrar, fioPararExecProg, fioContinuarExecProg;
wire [3:0] fioNumProcessoAnt, fioNumProcesso;
wire [4:0] fioContador;
wire [31:0] fioPCGerenciadorDeProcessos;
wire [1:0] FioFilaProcessos; 
wire execStore, execLoad, comecaProc;

wire parar;
wire [15:0] entradaParaLCD;
wire [31:0] pcAntigo;

assign numeroInstrucao = fioSaidaPC;

DivFreq b2v_inst4(
.Clock(CLOCK_50), 
.saidaClk(fioclock), 
.saidaClkWrite(fioclockwrite));

assign saidaclock1 = fioclock;
assign saidaclock2 = fioclockwrite;

GerenciadorDeProcessos	b2v_inst1(
	.clock(fioclockwrite),
	.parar(parar),
//	.chaveGPIO_IN(GPIO_IN[17]),
   .chaveGPIO_IN(1'b1), // enter sempre ativo
	.entradaParaLCD(entradaParaLCD),
	.pcAntigo(pcAntigo),
	.FilaProcessos(FioFilaProcessos),
	.NumProcesso(fioNumProcesso),
	.NumProcessoAnt(fioNumProcessoAnt),
	.Instrucao(fioInstrucao),
	.Encerrar(fioEncerrar),
	.contador(fioContador),
	.EnderecoPC(fioPCGerenciadorDeProcessos),
	.SemPreemp(fioSemPreemp),
	.infoMux4(fioSaidaMux4EntradaPC),
	.execStore(execStore),
	.execLoad(execLoad)); 
	
ProgramCounter	b2v_inst(
	.Entrada(fioPCGerenciadorDeProcessos),
	.Saida(fioSaidaPC));
	
MUX5	b2v_inst10(
	.EntradaExtensor(fioExtensor),
	.EntradaMemoriaDeDados(fioMemoriaDeDados),
	.EntradaSomadorPC(fioSomadorPC),
	.EntradaULA(fioResultadoULA),
	.MemToReg(fioSinalControleMemToReg),
	.SaidaMUX5(fioSaidaMux5));


SomadorPC	b2v_inst11(
	.Entrada(fioSaidaPC),
	.Saida(fioSomadorPC));


Somador	b2v_inst12(
	.Entrada1(fioSomadorPC),
	.Entrada2(fioExtensor),
	.Saida(fioSaidaSomadorPC));


ExtensorDeSinal2	b2v_inst13(
	.Instrucao(fioInstrucao),
	.Saida(fioSaidaExtensorDeSinal2));


MUX3	b2v_inst14(
	.zero(fioSinalControleZero),
	.beqOp(fioSinalControleBeqOp),
	.bneqOp(fioSinalControleBneqOp),
	.EntradaSomador(fioExtensor),
	.EntradaSomadorPC(fioSomadorPC),
	.SaidaMUX3(fioSaidaMux3));


MUX4	b2v_inst15(
	.Entradajr(fioDado1BR),
	.EntradaJump(fioSaidaExtensorDeSinal2),
	.EntradaMUX3(fioSaidaMux3),
	.JumpOp(fioSinalControleJumpOp),
	.SaidaMUX4(fioSaidaMux4EntradaPC));


MUX6	b2v_inst17(
	.In(fioSinalControleIn),
	.Out(fioSinalControleOut),
	.EntradaFPGA(fioentradaFPGA),
	.fioDado1BR(fioDado1BR),
	.EntradaMUX5(fioSaidaMux5),
	.DadoASerEscrito(fioDadoParaEscrita),
	.Saida(fioSaidaMux6DadoEscritaBR));


MemoriaDeInstrucoes	b2v_inst2(
	.Clock(fioclockwrite),
	.Endereco(fioSaidaPC),
	.Instrucao(fioInstrucao));


UnidadeDeControle	b2v_inst29(
	.Instrucao(fioInstrucao),
	.beqOp(fioSinalControleBeqOp),
	.bneqOp(fioSinalControleBneqOp),
	.MemDadosLeitura(fioSinalControleMemDadosLeitura),
	.MemDadosEscrita(fioSinalControleMemDadosEscrita),
	.ULASrc(fioSinalControleUlaSrc),
	.Size(fioSinalControleSize),
	.EscritaBDR(fioSinalControleEscritaBDR),
	.In(fioSinalControleIn),
	.Out(fioSinalControleOut),
	.JumpOp(fioSinalControleJumpOp),
	.MemToReg(fioSinalControleMemToReg),
	.RegDest(fioSinalControleRegDest),
	.ULAOp(fioSinalControleUlaOp),
	.Halt(fioSinalControleHalt),
	.Reset(fioSinalControleReset),
	.MemInstrucoesLeitura(fioSinalControleMemInstrucoesLeitura),
	.MemInstrucoesEscrita(fioSinalControleMemInstrucoesEscrita)
	);


BancoDeRegistradores	b2v_inst3(
	.Clock(fioclockwrite),
	.EscritaBDR(fioSinalControleEscritaBDR),
	.DadoParaEscrita(fioDadoParaEscrita),
	.Instrucao(fioInstrucao),
	.RegEscrita(fioRegParaEscrita),
	.Dado1(fioDado1BR),
	.Dado2(fioDado2BR));


MUX1	b2v_inst5(
	.Instrucao(fioInstrucao),
	.RegDest(fioSinalControleRegDest),
	.RegEscrita(fioRegParaEscrita));


ExtensorDeSinal	b2v_inst6(
	.Size(fioSinalControleSize),
	.Instrucao(fioInstrucao),
	.Saida(fioExtensor));


MUX2	b2v_inst7(
	.ULASrc(fioSinalControleUlaSrc),
	.Dado2(fioDado2BR),
	.DadoExtensor(fioExtensor),
	.Operando2(fioOperando2));

ULA	b2v_inst8(
	.Operando1(fioDado1BR),
	.Operando2(fioOperando2),
	.ULAOp(fioSinalControleUlaOp),
	.zero(fioSinalControleZero),
	.Resultado(fioResultadoULA));


MemoriaDeDados	b2v_inst9(
	.clock(fioclock),
	.writeMemory(fioSinalControleMemDadosEscrita),
	.ReadMemory(fioSinalControleMemDadosLeitura),
	.clockWrite(fioclockwrite),
	.dadoEscrita(fioDado2BR),
	.endereco(fioResultadoULA),
	.dadoLeitura(fioMemoriaDeDados));
	
	
ConversorBCD	b2v_inst40(
	.Entrada(fioSaidaMux6DadoEscritaBR),
	.a(fioEntradaA),
	.b(fioEntradaB),
	.c(fioEntradaC),
	.d(fioEntradaD),
	.e(fioEntradaE),
	.f(fioEntradaF),
	.g(fioEntradaG),
	.h(fioEntradaH));


Display7	b2v_inst46(
	.Entrada(fioEntradaA),
	.Inst(fioInstrucao),
	.HEX7(HEX7));


Display6	b2v_inst47(
	.Entrada(fioEntradaB),
	.Inst(fioInstrucao),
	.HEX6(HEX6));


Display5	b2v_inst48(
	.Entrada(fioEntradaC),
	.Inst(fioInstrucao),
	.HEX5(HEX5));


Display4	b2v_inst51(
	.Entrada(fioEntradaD),
	.Inst(fioInstrucao),
	.HEX4(HEX4));


Display3	b2v_inst52(
	.Entrada(fioEntradaE),
	.Inst(fioInstrucao),
	.HEX3(HEX3));


Display2	b2v_inst54(
	.Entrada(fioEntradaF),
	.Inst(fioInstrucao),
	.HEX2(HEX2));


Display1	b2v_inst55(
	.Entrada(fioEntradaG),
	.Inst(fioInstrucao),
	.HEX1(HEX1));


Display0	b2v_inst57(
	.Entrada(fioEntradaH),
	.Inst(fioInstrucao),
	.HEX0(HEX0));

Entrada	b2v_inst50(
	 .Clock(CLOCK_50),
    .GPIO_IN(GPIO_IN),
	 .Dado(fioentradaFPGA),
	 .UART_TXD(UART_TXD));

SaidaEsquematico	b2v_inst37(
	.Entrada(fioSaidaMux6DadoEscritaBR),
	.Saida(Saida));
	
LCD lcd(
  CLOCK_50, 
  fioclockwrite, 
  lcd_trd_msg,   //    50 MHz clock
  8'd0,    //    Toggle GPIO_INitch[17:0]
  entradaParaLCD,
  LCD_ON,    // LCD Power ON/OFF
  LCD_BLON,    // LCD Back Light ON/OFF
  LCD_RW,    // LCD Read/Write Select, 0 = Write, 1 = Read
  LCD_EN,    // LCD Enable
  LCD_RS,    // LCD Command/Data Select, 0 = Command, 1 = Data
  LCD_DATA    // LCD Data bus 8 bits
);
endmodule


module DivFreq(Clock, saidaClk, saidaClkWrite);
input Clock;
output reg saidaClk, saidaClkWrite;
reg [25:0] auxClk;
reg [28:0] auxClkWrite;

always @ (posedge Clock)
    if (auxClk == 26'd5000000)
	begin
	    auxClk <= 26'd0;
	    saidaClk <= 1;
    end
    else
    begin
	    auxClk<= auxClk+1;
	    saidaClk <= 0;
    end

always @ (posedge Clock)
    if (auxClkWrite == 29'd10000000)
	begin
	    auxClkWrite <= 29'd0;
	    saidaClkWrite <= 1;
    end
    else
    begin
	    auxClkWrite<= auxClkWrite+1;
	    saidaClkWrite <= 0;
    end
endmodule


module BancoDeRegistradores (Clock, EscritaBDR, Instrucao, RegEscrita, DadoParaEscrita, Dado1, Dado2);
   input wire EscritaBDR, Clock;
   input wire [3:0] RegEscrita; 
	input [31:0] Instrucao;
	wire [3:0] Reg1Leitura, Reg2Leitura;
	assign Reg1Leitura = Instrucao[25:21];
	assign Reg2Leitura = Instrucao[20:16];
   input wire [31:0] DadoParaEscrita;
   output [31:0] Dado1, Dado2;

   reg [31:0] Registradores[33:0]; // 31 

   integer i;

   always @ (posedge Clock) 
      begin
       if (EscritaBDR == 1)
            Registradores[RegEscrita] = DadoParaEscrita;
      end

   assign Dado1 = Registradores[Reg1Leitura];
   assign Dado2 = Registradores[Reg2Leitura];
endmodule


module Clock(Clock, clockWrite);
	input Clock;
	output clockWrite; 

	reg[27:0] count = 28'd0;
	parameter divisor = 28'd60; //quando for 60

	always @(posedge Clock)
		begin
			 count <= count + 28'd1;
			 
			 if(count >= (divisor-1))
				begin
					count <= 28'd0;
				end
		end
		
	assign clockWrite = (count < divisor/2) ? 1'b0 : 1'b1;
		
endmodule

module ExtensorDeSinal (Size, Instrucao, Saida);
   input wire Size;
	input wire [31:0] Instrucao;
	wire [15:0] DezesseisBits;
	wire [20:0] VinteEUmBits;
	assign DezesseisBits = Instrucao[15:0];
	assign VinteEUmBits = Instrucao[20:0];
   output reg [31:0] Saida;

   always @ (Size)
      case(Size)
         1'b0:
            begin
               Saida = {16'b0, DezesseisBits};
            end
         1'b1:
            begin
               Saida = {11'b0, VinteEUmBits};
            end
      endcase
endmodule


module SaidaEsquematico (Entrada, Saida);
	input [31:0] Entrada;
	output [31:0] Saida;
	
	assign Saida = Entrada;
 endmodule


module ExtensorDeSinal2 (Instrucao, Saida);
	input wire [31:0] Instrucao;
	wire [25:0] EnderecoJump;
	assign EnderecoJump = Instrucao[25:0];
   output [31:0] Saida;

   assign Saida = {6'b0, EnderecoJump};
endmodule



module MemoriaDeDados(
	input [31:0] dadoEscrita,
	input [31:0] endereco,
	input writeMemory, ReadMemory, clockWrite, clock,
	
	output reg [31:0] dadoLeitura
);

	reg [31:0] memRam[2048 * 11:0];
	
	always @ (posedge clock)
		begin
			if (ReadMemory)
				begin
					dadoLeitura = memRam[endereco];
				end
		end
	
	always @ (negedge clockWrite)
		begin
			if (writeMemory)
				begin
					memRam[endereco] = dadoEscrita;
				end
		end
	
endmodule


module ULA (Operando1, Operando2, ULAOp, Resultado, zero);
	input wire [31:0] Operando1, Operando2;
	input wire [3:0] ULAOp;
	output reg [31:0] Resultado;
	output reg zero;
	
	always @ (*)
		case(ULAOp)
			4'b0000:
				begin
					Resultado = Operando1 + Operando2;
					zero = 1'b0;
				end
			4'b0001:
				begin
					Resultado = Operando1 - Operando2;
					zero = 1'b0;
				end
			4'b0010:
				begin
					Resultado = Operando1 * Operando2;
					zero = 1'b0;
				end
			4'b0011:
				begin
					Resultado = Operando1 / Operando2;
					zero = 1'b0;
				end
			4'b0100:
				begin
					Resultado = Operando1 & Operando2;
					zero = 1'b0;
				end
			4'b0101:
				begin
					Resultado = Operando1 | Operando2;
					zero = 1'b0;
				end
			4'b0110:
				begin
					Resultado = Operando1 ^ Operando2;
					zero = 1'b0;
				end
			4'b0111:
				begin
					Resultado = ~Operando1;
					zero = 1'b0;
				end
			4'b1000:
				begin
					Resultado = Operando1 >> Operando2;
					zero = 1'b0;
				end
			4'b1001:
				begin
					Resultado = Operando1 << Operando2;	
					zero = 1'b0;
				end
			4'b1010:
				begin
					zero = Operando1 == Operando2;
					Resultado = Operando1 == Operando2;
				end
			4'b1011:
				begin
					if (Operando1 != Operando2)
						begin
							zero = 1'b0;
						end
						else
						begin
							zero = 1'b1;
						end
					Resultado = Operando1 != Operando2;
				end
			4'b1100:
				begin
					Resultado = Operando1 < Operando2;
					zero = 1'b0;
				end
			default:
            begin
					Resultado = 32'd0;
					zero = 1'b0;
				end
		endcase
endmodule


module SomadorPC (Entrada, Saida);
	input wire [31:0] Entrada;
   output [31:0] Saida;

   assign Saida = Entrada + 32'b1;
 
endmodule

module Somador (Entrada1, Entrada2, Saida);
	input wire [31:0] Entrada1, Entrada2;
   output [31:0] Saida;

   assign Saida = Entrada1 + Entrada2;
endmodule

module MUX1 (RegDest, Instrucao, RegEscrita);
   input wire [1:0] RegDest;
   input wire [31:0] Instrucao;
	wire [3:0] E1, E2, E3;
	assign E1 = Instrucao[25:21];
	assign E2 = Instrucao[20:16];
	assign E3 = Instrucao[15:11];
   output reg [3:0] RegEscrita;


   always @ (*)
      case(RegDest)
         2'b00:
            begin
               RegEscrita = E1;
            end
         2'b01:
            begin
               RegEscrita = E2;
            end
         2'b10:
            begin
               RegEscrita = E3;
            end
          2'b11:
       		begin
               RegEscrita = 5'b11111;
            end
      endcase
endmodule

module MUX6 (In, Out, EntradaFPGA, EntradaMUX5, DadoASerEscrito, fioDado1BR, Saida);
	input wire In, Out;
	input wire [16:0] EntradaFPGA;
	input wire [31:0] EntradaMUX5, fioDado1BR;
   output reg [31:0] DadoASerEscrito, Saida;
	
	reg [31:0] temp;
			
	always @ (*)
		begin
			case(In)
				1'b0: 
					begin
						DadoASerEscrito = EntradaMUX5;
					end
				1'b1:
					begin
						DadoASerEscrito = {15'b0, EntradaFPGA};
					end
			endcase
			
			case(Out)
				1'b0:
					begin
						Saida = 32'd0;
					end
				1'b1:
					begin
						if(In == 1)
							begin
								Saida = {15'b0, EntradaFPGA};
							end
						else
							begin	
								Saida = fioDado1BR;
							end 
					end
			endcase
		end 
endmodule

module MUX5 (MemToReg, EntradaSomadorPC, EntradaMemoriaDeDados, EntradaULA, EntradaExtensor, SaidaMUX5);
	input wire [1:0] MemToReg;
	input wire [31:0] EntradaSomadorPC, EntradaMemoriaDeDados, EntradaULA, EntradaExtensor;
   output reg [31:0] SaidaMUX5;
			
	always @ (*)
		begin
				case(MemToReg)
					2'b00:
						begin
							SaidaMUX5 = EntradaSomadorPC;
						end
					2'b01:
						begin
							SaidaMUX5 = EntradaMemoriaDeDados;
						end
					2'b10:
						begin
							SaidaMUX5 = EntradaULA;
						end
					2'b11:
						begin
							SaidaMUX5 = EntradaExtensor;
						end
				endcase
		end
endmodule

module MUX4 (JumpOp, EntradaMUX3, EntradaJump, Entradajr, SaidaMUX4);
	input wire [1:0] JumpOp;
	input wire [31:0] EntradaMUX3, EntradaJump, Entradajr;
   output reg [31:0] SaidaMUX4;
			
	always @ (*)
		begin
			case(JumpOp)
				2'b00:
					begin
						SaidaMUX4 = EntradaJump;
					end
				2'b01:
					begin
						SaidaMUX4 = EntradaMUX3;
					end
				2'b10:
					begin
						SaidaMUX4 = Entradajr;
					end
				default:
					begin
						SaidaMUX4 = 31'b0;
					end
			endcase
		end 
endmodule

module MUX3 (zero, beqOp, bneqOp, EntradaSomadorPC, EntradaSomador, SaidaMUX3);
	input wire zero, beqOp, bneqOp;
	input wire [31:0] EntradaSomadorPC, EntradaSomador;
   output reg [31:0] SaidaMUX3;
	
	reg OpMUX3;
		
	always @ (*)
	
		if ((beqOp == 1 && zero == 1) || (bneqOp == 1 && zero == 0))
			begin
				OpMUX3 = 1; 
			end

		else 
			begin
				OpMUX3 = 0; 
			end
		
	always @ (*)
		begin
			case(OpMUX3)
				1'b0:
					begin
						SaidaMUX3 = EntradaSomadorPC;
					end
				1'b1:
					begin
						SaidaMUX3 = EntradaSomador;
					end
			endcase
		end 
endmodule

module MUX2 (ULASrc, Dado2, DadoExtensor, Operando2);
	input wire ULASrc;
	input wire [31:0] Dado2, DadoExtensor;
   output reg [31:0] Operando2;


	always @ (*)
		case(ULASrc)
			2'b0:
				begin
					Operando2 = Dado2;
				end
			2'b1:
				begin
					Operando2 = DadoExtensor;
				end
		endcase
endmodule


module UnidadeDeControle (Instrucao, RegDest, beqOp, bneqOp, MemToReg, MemDadosLeitura, MemDadosEscrita, JumpOp, ULASrc, Size, EscritaBDR, In, Out, ULAOp, 
Halt, Reset, MemInstrucoesLeitura, MemInstrucoesEscrita);
	input wire [31:0] Instrucao;
	wire [5:0] Opcode;
	assign Opcode = Instrucao[31:26];
	
	output reg beqOp, bneqOp, MemDadosLeitura, MemDadosEscrita, ULASrc, Size, EscritaBDR, In, Out, Halt, Reset, MemInstrucoesLeitura, MemInstrucoesEscrita;
	output reg [3:0] ULAOp;
	output reg [1:0] RegDest, MemToReg, JumpOp;
	
	reg FimProg;
	
	
	always @ (*)
		case(Opcode)
			6'b000000: //add
				begin
					RegDest = 2'b10;
					beqOp = 1'b0;
					bneqOp = 1'b0;
					MemToReg = 2'b10;
					MemDadosLeitura = 1'b0;
					MemDadosEscrita = 1'b0;
					JumpOp = 2'b01;
					ULAOp = 4'b0000;
					ULASrc = 1'b0;
					Size = 1'b0;
					EscritaBDR = 1'b1;
					In = 1'b0;
					Out = 1'b1;
					
					Halt = 1'b0;
					Reset = 1'b0;
				end
			6'b000001: //addi
				begin
					RegDest = 2'b01;
					beqOp = 1'b0;
					bneqOp = 1'b0;
					MemToReg = 2'b10;
					MemDadosLeitura = 1'b0;
					MemDadosEscrita = 1'b0;
					JumpOp = 2'b01;
					ULAOp = 4'b0000;
					ULASrc = 1'b1;
					Size = 1'b0;
					EscritaBDR = 1'b1;
					In = 1'b0;
					Out = 1'b1;
					
					Halt = 1'b0;
					Reset = 1'b0;
				end
			6'b000010: //sub
				begin
					RegDest = 2'b10;
					beqOp = 1'b0;
					bneqOp = 1'b0;
					MemToReg = 2'b10;
					MemDadosLeitura = 1'b0;
					MemDadosEscrita = 1'b0;
					JumpOp = 2'b01;
					ULAOp = 4'b0001;
					ULASrc = 1'b0;
					Size = 1'b0;
					EscritaBDR = 1'b1;
					In = 1'b0;
					Out = 1'b1;
					
					Halt = 1'b0;
					Reset = 1'b0;
				end
			6'b000011: //subi
				begin
					RegDest = 2'b01;
					beqOp = 1'b0;
					bneqOp = 1'b0;
					MemToReg = 2'b10;
					MemDadosLeitura = 1'b0;
					MemDadosEscrita = 1'b0;
					JumpOp = 2'b01;
					ULAOp = 4'b0001;
					ULASrc = 1'b1;
					Size = 1'b0;
					EscritaBDR = 1'b1;
					In = 1'b0;
					Out = 1'b1;
					
					Halt = 1'b0;
					Reset = 1'b0;
				end
			6'b000100: //mult
				begin
					RegDest = 2'b10;
					beqOp = 1'b0;
					bneqOp = 1'b0;
					MemToReg = 2'b10;
					MemDadosLeitura = 1'b0;
					MemDadosEscrita = 1'b0;
					JumpOp = 2'b01;
					ULAOp = 4'b0010;
					ULASrc = 1'b0;
					Size = 1'b0;
					EscritaBDR = 1'b1;
					In = 1'b0;
					Out = 1'b1;
					
					Halt = 1'b0;
					Reset = 1'b0;
				end
			6'b000101: //multi
				begin
					RegDest = 2'b01;
					beqOp = 1'b0;
					bneqOp = 1'b0;
					MemToReg = 2'b10;
					MemDadosLeitura = 1'b0;
					MemDadosEscrita = 1'b0;
					JumpOp = 2'b01;
					ULAOp = 4'b0010;
					ULASrc = 1'b1;
					Size = 1'b0;
					EscritaBDR = 1'b1;
					In = 1'b0;
					Out = 1'b1;
					
					Halt = 1'b0;
					Reset = 1'b0;
				end
			6'b000110: //div
				begin
					RegDest = 2'b10;
					beqOp = 1'b0;
					bneqOp = 1'b0;
					MemToReg = 2'b10;
					MemDadosLeitura = 1'b0;
					MemDadosEscrita = 1'b0;
					JumpOp = 2'b01;
					ULAOp = 4'b0011;
					ULASrc = 1'b0;
					Size = 1'b0;
					EscritaBDR = 1'b1;
					In = 1'b0;
					Out = 1'b1;
					
					Halt = 1'b0;
					Reset = 1'b0;
				end
			6'b000111: //divi
				begin
					RegDest = 2'b01;
					beqOp = 1'b0;
					bneqOp = 1'b0;
					MemToReg = 2'b10;
					MemDadosLeitura = 1'b0;
					MemDadosEscrita = 1'b0;
					JumpOp = 2'b01;
					ULAOp = 4'b0011;
					ULASrc = 1'b1;
					Size = 1'b0;
					EscritaBDR = 1'b1;
					In = 1'b0;
					Out = 1'b1;
					
					Halt = 1'b0;
					Reset = 1'b0;
				end
			6'b001000:  //load
				begin
					RegDest = 2'b01;
					beqOp = 1'b0;
					bneqOp = 1'b0;
					MemToReg = 2'b01;
					MemDadosLeitura = 1'b1;
					MemDadosEscrita = 1'b0;
					JumpOp = 2'b01;
					ULAOp = 4'b0000;
					ULASrc = 1'b1;
					Size = 1'b0;
					EscritaBDR = 1'b1;
					In = 1'b0;
					Out = 1'b1;
					
					Halt = 1'b0;
					Reset = 1'b0;
				end
			6'b001001: //loadi
				begin
					RegDest = 2'b00;
					beqOp = 1'b0;
					bneqOp = 1'b0;
					MemToReg = 2'b11;
					MemDadosLeitura = 1'b0;
					MemDadosEscrita = 1'b0;
					JumpOp = 2'b01;
					ULAOp = 4'b1111;
					ULASrc = 1'b0;
					Size = 1'b1;
					EscritaBDR = 1'b1;
					In = 1'b0;
					Out = 1'b1;
					
					Halt = 1'b0;
					Reset = 1'b0;
				end
			6'b001010:  //store
				begin
					RegDest = 2'b01;
					beqOp = 1'b0;
					bneqOp = 1'b0;
					//MemToReg = 2'b00;
					MemToReg = 2'b10; // dont care
					MemDadosLeitura = 1'b0;
					MemDadosEscrita = 1'b1;
					JumpOp = 2'b01;
					ULAOp = 4'b0000;
					//ULASrc = 1'b0;
					ULASrc = 1'b1;
					Size = 1'b0;
					EscritaBDR = 1'b0;
					In = 1'b0;
					Out = 1'b1;
					
					Halt = 1'b0;
					Reset = 1'b0;
				end
			6'b001011:  //and
				begin
					RegDest = 2'b10;
					beqOp = 1'b0;
					bneqOp = 1'b0;
					MemToReg = 2'b10;
					MemDadosLeitura = 1'b0;
					MemDadosEscrita = 1'b0;
					JumpOp = 2'b01;
					ULAOp = 4'b0100;
					ULASrc = 1'b0;
					Size = 1'b0;
					EscritaBDR = 1'b1;
					In = 1'b0;
					Out = 1'b1;
					
					Halt = 1'b0;
					Reset = 1'b0;
				end
			6'b001100: //or 
				begin
					RegDest = 2'b10;
					beqOp = 1'b0;
					bneqOp = 1'b0;
					MemToReg = 2'b10;
					MemDadosLeitura = 1'b0;
					MemDadosEscrita = 1'b0;
					JumpOp = 2'b01;
					ULAOp = 4'b0101;
					ULASrc = 1'b0;
					Size = 1'b0;
					EscritaBDR = 1'b1;
					In = 1'b0;
					Out = 1'b1;
					
					Halt = 1'b0;
					Reset = 1'b0;
				end
			6'b001101: //xor 
				begin
					RegDest = 2'b10;
					beqOp = 1'b0;
					bneqOp = 1'b0;
					MemToReg = 2'b10;
					MemDadosLeitura = 1'b0;
					MemDadosEscrita = 1'b0;
					JumpOp = 2'b01;
					ULAOp = 4'b0110;
					ULASrc = 1'b0;
					Size = 1'b0;
					EscritaBDR = 1'b1;
					In = 1'b0;
					Out = 1'b1;
					
					Halt = 1'b0;
					Reset = 1'b0;
				end
			6'b001110: //not 
				begin
					RegDest = 2'b10;
					beqOp = 1'b0;
					bneqOp = 1'b0;
					MemToReg = 2'b10;
					MemDadosLeitura = 1'b0;
					MemDadosEscrita = 1'b0;
					JumpOp = 2'b01;
					ULAOp = 4'b0111;
					ULASrc = 1'b0;
					Size = 1'b0;
					EscritaBDR = 1'b1;
					In = 1'b0;
					Out = 1'b1;
					
					Halt = 1'b0;
					Reset = 1'b0;
				end
			6'b001111: //shiftr
				begin
					RegDest = 2'b01;
					beqOp = 1'b0;
					bneqOp = 1'b0;
					MemToReg = 2'b10;
					MemDadosLeitura = 1'b0;
					MemDadosEscrita = 1'b0;
					JumpOp = 2'b01;
					ULAOp = 4'b1000;
					ULASrc = 1'b1;
					Size = 1'b0;
					EscritaBDR = 1'b1;
					In = 1'b0;
					Out = 1'b1;
					
					Halt = 1'b0;
					Reset = 1'b0;
				end
			6'b010000: //shiftl
				begin
					RegDest = 2'b01;
					beqOp = 1'b0;
					bneqOp = 1'b0;
					MemToReg = 2'b10;
					MemDadosLeitura = 1'b0;
					MemDadosEscrita = 1'b0;
					JumpOp = 2'b01;
					ULAOp = 4'b1001;
					ULASrc = 1'b1;
					Size = 1'b0;
					EscritaBDR = 1'b1;
					In = 1'b0;
					Out = 1'b1;
					
					Halt = 1'b0;
					Reset = 1'b0;
				end
			6'b010001: //beq
				begin
					RegDest = 2'b00;
					beqOp = 1'b1;
					bneqOp = 1'b0;
					MemToReg = 2'b10;
					MemDadosLeitura = 1'b0;
					MemDadosEscrita = 1'b0;
					JumpOp = 2'b01;
					ULAOp = 4'b1010;
					ULASrc = 1'b0;
					Size = 1'b0;
					EscritaBDR = 1'b0;
					In = 1'b0;
					Out = 1'b1;
					
					Halt = 1'b0;
					Reset = 1'b0;
				end
			6'b010010: //bneq
				begin
					RegDest = 2'b00;
					beqOp = 1'b0;
					bneqOp = 1'b1;
					MemToReg = 2'b10;
					MemDadosLeitura = 1'b0;
					MemDadosEscrita = 1'b0;
					JumpOp = 2'b01;
					ULAOp = 4'b1011;
					ULASrc = 1'b0;
					Size = 1'b0;
					EscritaBDR = 1'b0;
					In = 1'b0;
					Out = 1'b1;
					
					Halt = 1'b0;
					Reset = 1'b0;
				end
			6'b010011: //slt
				begin
					RegDest = 2'b10;
					beqOp = 1'b0;
					bneqOp = 1'b0;
					MemToReg = 2'b10;
					MemDadosLeitura = 1'b0;
					MemDadosEscrita = 1'b0;
					JumpOp = 2'b01;
					ULAOp = 4'b1100;
					ULASrc = 1'b0;
					Size = 1'b0;
					EscritaBDR = 1'b1;
					In = 1'b0;
					Out = 1'b1;
					
					Halt = 1'b0;
					Reset = 1'b0;
				end
			6'b010100: //slti 
				begin
					RegDest = 2'b01;
					beqOp = 1'b0;
					bneqOp = 1'b0;
					MemToReg = 2'b10;
					MemDadosLeitura = 1'b0;
					MemDadosEscrita = 1'b0;
					JumpOp = 2'b01;
					ULAOp = 4'b1100;
					ULASrc = 1'b1;
					Size = 1'b0;
					EscritaBDR = 1'b1;
					In = 1'b0;
					Out = 1'b1;
					
					Halt = 1'b0;
					Reset = 1'b0;
				end
			6'b010101: //jump
				begin
					RegDest = 2'b11;
					beqOp = 1'b0;
					bneqOp = 1'b0;
					MemToReg = 2'b00;
					MemDadosLeitura = 1'b0;
					MemDadosEscrita = 1'b0;
					JumpOp = 2'b00;
					ULAOp = 4'b1111;
					ULASrc = 1'b0;
					Size = 1'b0;
					EscritaBDR = 1'b0;
					In = 1'b0;
					Out = 1'b0;
					
					Halt = 1'b0;
					Reset = 1'b0;
				end
			6'b010110: //jr
				begin
					RegDest = 2'b01;
					beqOp = 1'b0;
					bneqOp = 1'b0;
					MemToReg = 2'b11;
					MemDadosLeitura = 1'b0;
					MemDadosEscrita = 1'b0;
					JumpOp = 2'b10;
					ULAOp = 4'b1111;
					ULASrc = 1'b0;
					Size = 1'b1;
					EscritaBDR = 1'b0;
					In = 1'b0;
					Out = 1'b0;
					
					Halt = 1'b0;
					Reset = 1'b0;
				end
			6'b010111: //jal
				begin
					RegDest = 2'b11;
					beqOp = 1'b0;
					bneqOp = 1'b0;
					MemToReg = 2'b00;
					MemDadosLeitura = 1'b0;
					MemDadosEscrita = 1'b0;
					JumpOp = 2'b00;
					ULAOp = 4'b1111;
					ULASrc = 1'b0;
					Size = 1'b0;
					EscritaBDR = 1'b1;
					In = 1'b0;
					Out = 1'b0;
					
					Halt = 1'b0;
					Reset = 1'b0;
				end
				6'b011000: //In
				begin
					RegDest = 2'b00;
					beqOp = 1'b0;
					bneqOp = 1'b0;
					MemToReg = 2'b00;
					MemDadosLeitura = 1'b0;
					MemDadosEscrita = 1'b0;
					JumpOp = 2'b01;
					ULAOp = 4'b1111;
					ULASrc = 1'b0;
					Size = 1'b0;
					EscritaBDR = 1'b1;
					In = 1'b1;
					Out = 1'b1;
					
					Halt = 1'b0;
					Reset = 1'b0;
				end
				
				6'b011001: //out
				begin
					RegDest = 2'b01;
					beqOp = 1'b0;
					bneqOp = 1'b0;
					MemToReg = 2'b10;
					MemDadosLeitura = 1'b0;
					MemDadosEscrita = 1'b0;
					JumpOp = 2'b01;
					ULAOp = 4'b0000;
					ULASrc = 1'b1;
					Size = 1'b0;
					EscritaBDR = 1'b1;
					In = 1'b0;
					Out = 1'b1;
					
					Halt = 1'b0;
					Reset = 1'b0;
				end
				
				6'b111111: //END
				begin
					RegDest = 2'b00;
					beqOp = 1'b0;
					bneqOp = 1'b0;
					MemToReg = 2'b00;
					MemDadosLeitura = 1'b0;
					MemDadosEscrita = 1'b0;
					JumpOp = 2'b01;
					ULAOp = 4'b1111;
					ULASrc = 1'b0;
					Size = 1'b0;
					EscritaBDR = 1'b0;
					In = 1'b0;
					Out = 1'b1;
					
					Halt = 1'b0;
					Reset = 1'b0;
				end
				
				6'b111110: // FimProg
				begin
					RegDest = 2'b00;
					beqOp = 1'b0;
					bneqOp = 1'b0;
					MemToReg = 2'b00;
					MemDadosLeitura = 1'b0;
					MemDadosEscrita = 1'b0;
					JumpOp = 2'b01;
					ULAOp = 4'b1111;
					ULASrc = 1'b0;
					Size = 1'b0;
					EscritaBDR = 1'b0;
					In = 1'b0;
					Out = 1'b1;
					
					Halt = 1'b0;
					Reset = 1'b0;
				end
				
				6'b111000: // AlternarParaProc
				begin
					RegDest = 2'b00;
					beqOp = 1'b0;
					bneqOp = 1'b0;
					MemToReg = 2'b00;
					MemDadosLeitura = 1'b0;
					MemDadosEscrita = 1'b0;
					JumpOp = 2'b01;
					ULAOp = 4'b1111;
					ULASrc = 1'b0;
					Size = 1'b0;
					EscritaBDR = 1'b0;
					In = 1'b0;
					Out = 1'b1;
					
					Halt = 1'b0;
					Reset = 1'b0;
				end
			
				6'b101111: //mensagem
				begin
					RegDest = 2'b01;
					beqOp = 1'b0;
					bneqOp = 1'b0;
					MemToReg = 2'b10;
					MemDadosLeitura = 1'b0;
					MemDadosEscrita = 1'b0;
					JumpOp = 2'b01;
					ULAOp = 4'b0000;
					ULASrc = 1'b1;
					Size = 1'b0;
					EscritaBDR = 1'b0;
					In = 1'b0;
					Out = 1'b0;
					
					Halt = 1'b0;
					Reset = 1'b0;
				end
		endcase
endmodule


module ConversorBCD (Entrada, a, b, c, d, e, f, g, h);
	input [31:0] Entrada ;
	output reg [3:0] a, b, c, d, e, f, g, h;
	
	integer i;

	always @ (*) 
		begin
			a = 4'd0 ;
			b = 4'd0 ;
			c = 4'd0 ;
			d = 4'd0 ;
			e = 4'd0 ;
			f = 4'd0 ;
			g = 4'd0 ;
			h = 4'd0 ;

			for (i = 31; i >=0; i = i -1) 
				begin
					if(a > 4)
						a = a + 3;
					if(b > 4)
						b = b + 3;
					if(c > 4)
						c = c + 3;
					if(d > 4)
						d = d + 3;
					if(e > 4)
						e = e + 3;
					if(f > 4)
						f = f + 3;
					if(g > 4)
						g = g + 3;
					if (h > 4)
						h = h + 3;
 
					a = a << 1;
					a[0] = b[3];
	
					b = b << 1;
					b[0] = c[3];

					c = c << 1;
					c[0] = d[3];

					d = d << 1;
					d[0] = e[3];

					e = e << 1;
					e[0] = f[3];

					f = f << 1;
					f[0] = g[3];

					g = g << 1;
					g[0] = h[3];

					h = h << 1;
					h[0] = Entrada[i];
				end
		end
 endmodule



module Display0 (Entrada, Inst, HEX0);
	input wire [3:0]Entrada;
	input wire [31:0] Inst;
	
	wire [5:0] Instrucao;
	assign Instrucao = Inst[31:26];

    output reg[0:6] HEX0;

    always @(*)
		if (Instrucao == 6'b001000) // LOAD
			begin
				HEX0 = 7'b1000010; // d				
			end
			
		else if (Instrucao == 6'b111111) // END
			begin
				HEX0 = 7'b0110000; // E
			end

			
		else if (Instrucao == 6'b001010) // STORE
			begin
				HEX0 = 7'b0010000; // e
			end
			
		else if (Instrucao == 6'b010101) // JUMP
			begin
				HEX0 = 7'b1000011; // J
			end
			
		else if (Instrucao == 6'b010111) // JAL
			begin
				HEX0 = 7'b1110001; // L
			end
			
		else if (Instrucao == 6'b010110) // JR
			begin
				HEX0 = 7'b0111001; // r
			end
		
		else
			begin
				case(Entrada)
				4'b0000: HEX0 = 7'b0000001;
				4'b0001: HEX0 = 7'b1001111;
				4'b0010: HEX0 = 7'b0010010;
				4'b0011: HEX0 = 7'b0000110;
				4'b0100: HEX0 = 7'b1001100;
				4'b0101: HEX0 = 7'b0100100;
				4'b0110: HEX0 = 7'b0100000;
				4'b0111: HEX0 = 7'b0001111;
				4'b1000: HEX0 = 7'b0000000;
				4'b1001: HEX0 = 7'b0000100;
				default: HEX0 = 7'b1111111;
				endcase
		 end
endmodule


module Display1 (Entrada, Inst, HEX1);
   input wire [3:0]Entrada;
	input wire [31:0] Inst;
	
	wire [5:0] Instrucao;
	assign Instrucao = Inst[31:26];

    output reg[0:6] HEX1;

   
	always @(*)
		if (Instrucao == 6'b001000) // LOAD
			begin
				HEX1 = 7'b0000010; // a			
			end
			
		else if (Instrucao == 6'b111111) // END
			begin
				HEX1 = 7'b1111111;
			end

			
		else if (Instrucao == 6'b001010) // STORE
			begin
				HEX1 = 7'b0111001; // r
			end
			
		else if (Instrucao == 6'b010101) // JUMP
			begin
				HEX1 = 7'b1111111; // - 
			end
			
		else if (Instrucao == 6'b010111) // JAL
			begin
				HEX1 = 7'b0000010; // a		
			end
			
		else if (Instrucao == 6'b010110) // JR
			begin
				HEX1 = 7'b1000011; // J
			end
		else
			begin
				case(Entrada)
				4'b0000: HEX1 = 7'b0000001;
				4'b0001: HEX1 = 7'b1001111;
				4'b0010: HEX1 = 7'b0010010;
				4'b0011: HEX1 = 7'b0000110;
				4'b0100: HEX1 = 7'b1001100;
				4'b0101: HEX1 = 7'b0100100;
				4'b0110: HEX1 = 7'b0100000;
				4'b0111: HEX1 = 7'b0001111;
				4'b1000: HEX1 = 7'b0000000;
				4'b1001: HEX1 = 7'b0000100;
				default: HEX1 = 7'b1111111;
				endcase
		 end
endmodule


module Display2 (Entrada, Inst, HEX2);
    input wire [3:0]Entrada;
	input wire [31:0] Inst;
	
	wire [5:0] Instrucao;
	assign Instrucao = Inst[31:26];

    output reg[0:6] HEX2;

   always @(*)
		if (Instrucao == 6'b001000) // LOAD
			begin
				HEX2 = 7'b0000001; // O		
			end
			
		else if (Instrucao == 6'b111111) // END
			begin
				HEX2 = 7'b1111111;
			end

			
		else if (Instrucao == 6'b001010) // STORE
			begin
				HEX2 = 7'b0000001; // O
			end
			
		else if (Instrucao == 6'b010101) // JUMP
			begin
				HEX2 = 7'b1111111; // - 
			end
			
		else if (Instrucao == 6'b010111) // JAL
			begin
				HEX2 = 7'b1000011; // J		
			end
			
		else if (Instrucao == 6'b010110) // JR
			begin
				HEX2 = 7'b1111111; // - 
			end
		else
			begin
				case(Entrada)
				4'b0000: HEX2 = 7'b0000001;
				4'b0001: HEX2 = 7'b1001111;
				4'b0010: HEX2 = 7'b0010010;
				4'b0011: HEX2 = 7'b0000110;
				4'b0100: HEX2 = 7'b1001100;
				4'b0101: HEX2 = 7'b0100100;
				4'b0110: HEX2 = 7'b0100000;
				4'b0111: HEX2 = 7'b0001111;
				4'b1000: HEX2 = 7'b0000000;
				4'b1001: HEX2 = 7'b0000100;
				default: HEX2 = 7'b1111111;
				endcase
		 end
endmodule



module Display3 (Entrada, Inst, HEX3);
   input wire [3:0]Entrada;
	input wire [31:0] Inst;
	
	wire [5:0] Instrucao;
	assign Instrucao = Inst[31:26];

    output reg[0:6] HEX3;

    always @(*)
		if (Instrucao == 6'b001000) // LOAD
			begin
				HEX3 = 7'b1110001; //L	
			end
			
		else if (Instrucao == 6'b111111) // END
			begin
				HEX3 = 7'b1111111;
			end

			
		else if (Instrucao == 6'b001010) // STORE
			begin
				HEX3 = 7'b1111001; // t
			end
			
		else if (Instrucao == 6'b010101) // JUMP
			begin
				HEX3 = 7'b1111111; // - 
			end
			
		else if (Instrucao == 6'b010111) // JAL
			begin
				HEX3 = 7'b1111111; // - 
			end
			
		else if (Instrucao == 6'b010110) // JR
			begin
				HEX3 = 7'b1111111; // - 
			end
		else
			begin
				case(Entrada)
				4'b0000: HEX3 = 7'b0000001;
				4'b0001: HEX3 = 7'b1001111;
				4'b0010: HEX3 = 7'b0010010;
				4'b0011: HEX3 = 7'b0000110;
				4'b0100: HEX3 = 7'b1001100;
				4'b0101: HEX3 = 7'b0100100;
				4'b0110: HEX3 = 7'b0100000;
				4'b0111: HEX3 = 7'b0001111;
				4'b1000: HEX3 = 7'b0000000;
				4'b1001: HEX3 = 7'b0000100;
				default: HEX3 = 7'b1111111;
				endcase
		 end
endmodule


module Display4 (Entrada, Inst, HEX4);
   input wire [3:0]Entrada;
	input wire [31:0] Inst;
	
	wire [5:0] Instrucao;
	assign Instrucao = Inst[31:26];

    output reg[0:6] HEX4;

    always @(*)
		if (Instrucao == 6'b001000) // LOAD
			begin
				HEX4 = 7'b1111111; // - 
			end
			
		else if (Instrucao == 6'b111111) // END
			begin
				HEX4 = 7'b1111111;
			end

			
		else if (Instrucao == 6'b001010) // STORE
			begin
				HEX4 = 7'b0100100; // S
			end
			
		else if (Instrucao == 6'b010101) // JUMP
			begin
				HEX4 = 7'b1111111; // - 
			end
			
		else if (Instrucao == 6'b010111) // JAL
			begin
				HEX4 = 7'b1111111; // - 
			end
			
		else if (Instrucao == 6'b010110) // JR
			begin
				HEX4 = 7'b1111111; // - 
			end
		else
			begin
				case(Entrada)
				4'b0000: HEX4 = 7'b0000001;
				4'b0001: HEX4 = 7'b1001111;
				4'b0010: HEX4 = 7'b0010010;
				4'b0011: HEX4 = 7'b0000110;
				4'b0100: HEX4 = 7'b1001100;
				4'b0101: HEX4 = 7'b0100100;
				4'b0110: HEX4 = 7'b0100000;
				4'b0111: HEX4 = 7'b0001111;
				4'b1000: HEX4 = 7'b0000000;
				4'b1001: HEX4 = 7'b0000100;
				default: HEX4 = 7'b1111111;
				endcase
		 end
endmodule


module Display5 (Entrada, Inst, HEX5);
    input wire [3:0]Entrada;
	input wire [31:0] Inst;
	
	wire [5:0] Instrucao;
	assign Instrucao = Inst[31:26];

    output reg[0:6] HEX5;

   always @(*)
		if (Instrucao == 6'b001000) // LOAD
			begin
				HEX5 = 7'b1111111; // - 
			end
			
		else if (Instrucao == 6'b111111) // END
			begin
				HEX5 = 7'b1111111;
			end

			
		else if (Instrucao == 6'b001010) // STORE
			begin
				HEX5 = 7'b1111111; // - 
			end
			
		else if (Instrucao == 6'b010101) // JUMP
			begin
				HEX5 = 7'b1111111; // - 
			end
			
		else if (Instrucao == 6'b010111) // JAL
			begin
				HEX5 = 7'b1111111; // - 
			end
			
		else if (Instrucao == 6'b010110) // JR
			begin
				HEX5 = 7'b1111111; // -  
			end
		else
			begin
				case(Entrada)
				4'b0000: HEX5 = 7'b0000001;
				4'b0001: HEX5 = 7'b1001111;
				4'b0010: HEX5 = 7'b0010010;
				4'b0011: HEX5 = 7'b0000110;
				4'b0100: HEX5 = 7'b1001100;
				4'b0101: HEX5 = 7'b0100100;
				4'b0110: HEX5 = 7'b0100000;
				4'b0111: HEX5 = 7'b0001111;
				4'b1000: HEX5 = 7'b0000000;
				4'b1001: HEX5 = 7'b0000100;
				default: HEX5 = 7'b1111111;
				endcase
		 end
endmodule


module Display6 (Entrada, Inst, HEX6);
   input wire [3:0]Entrada;
	input wire [31:0] Inst;
	
	wire [5:0] Instrucao;
	assign Instrucao = Inst[31:26];

    output reg[0:6] HEX6;

    always @(*)
		if (Instrucao == 6'b001000) // LOAD
			begin
				HEX6 = 7'b1111111; // - 
			end
			
		else if (Instrucao == 6'b111111) // END
			begin
				HEX6 = 7'b1111111;
			end

			
		else if (Instrucao == 6'b001010) // STORE
			begin
				HEX6 = 7'b1111111; // - 
			end
			
		else if (Instrucao == 6'b010101) // JUMP
			begin
				HEX6 = 7'b1111111; // - 
			end
			
		else if (Instrucao == 6'b010111) // JAL
			begin
				HEX6 = 7'b1111111; // - 
			end
			
		else if (Instrucao == 6'b010110) // JR
			begin
				HEX6 = 7'b1111111; // - 
			end
		else
			begin
				case(Entrada)
				4'b0000: HEX6 = 7'b0000001;
				4'b0001: HEX6 = 7'b1001111;
				4'b0010: HEX6 = 7'b0010010;
				4'b0011: HEX6 = 7'b0000110;
				4'b0100: HEX6 = 7'b1001100;
				4'b0101: HEX6 = 7'b0100100;
				4'b0110: HEX6 = 7'b0100000;
				4'b0111: HEX6 = 7'b0001111;
				4'b1000: HEX6 = 7'b0000000;
				4'b1001: HEX6 = 7'b0000100;
				default: HEX6 = 7'b1111111;
				endcase
		 end
endmodule


module Display7 (Entrada, Inst, HEX7);
    input wire [3:0]Entrada;
	input wire [31:0] Inst;
	
	wire [5:0] Instrucao;
	assign Instrucao = Inst[31:26];

    output reg[0:6] HEX7;

   always @(*)
		if (Instrucao == 6'b001000) // LOAD
			begin
				HEX7 = 7'b1111111; // - 
			end
			
		
		else if (Instrucao == 6'b111111) // END
			begin
				HEX7 = 7'b1111111;
			end
		
			
		else if (Instrucao == 6'b001010) // STORE
			begin
				HEX7 = 7'b1111111; // - 
			end
			
		else if (Instrucao == 6'b010101) // JUMP
			begin
				HEX7 = 7'b1111111; // - 
			end
			
		else if (Instrucao == 6'b010111) // JAL
			begin
				HEX7 = 7'b1111111; // - 
			end
			
		else if (Instrucao == 6'b010110) // JR
			begin
				HEX7 = 7'b1111111; // -  
			end
		else
			begin
				case(Entrada)
				4'b0000: HEX7 = 7'b0000001;
				4'b0001: HEX7 = 7'b1001111;
				4'b0010: HEX7 = 7'b0010010;
				4'b0011: HEX7 = 7'b0000110;
				4'b0100: HEX7 = 7'b1001100;
				4'b0101: HEX7 = 7'b0100100;
				4'b0110: HEX7 = 7'b0100000;
				4'b0111: HEX7 = 7'b0001111;
				4'b1000: HEX7 = 7'b0000000;
				4'b1001: HEX7 = 7'b0000100;
				default: HEX7 = 7'b1111111;
				endcase
		 end
endmodule


module uart_tx #(
    // Parâmetros do módulo:
    parameter CLK_FREQ  = 50_000_000, // Frequência do clock do sistema em Hz
    parameter BAUD_RATE = 9600        // Taxa de transmissão (p/ Arduino)
)(
    input wire          i_clk,         // Clock principal do sistema
    input wire          i_rst_n,       // Reset assíncrono, ativo em nível baixo
    input wire          i_tx_start,    // Pulso de 1 ciclo para iniciar a transmissão
    input wire  [7:0]   i_tx_data,     // Byte (8 bits) que será transmitido

    output reg          o_tx_serial,   // Linha de saída serial UART (TX)
    output reg          o_tx_busy      // Sinalizador de ocupação (indica transmissão em andamento)
);

//----------------------------------------------------------------------
// Cálculo de quantos ciclos de clock representam 1 bit UART
// Exemplo: Com clock de 50 MHz e baud rate de 9600, um bit leva ~5208 ciclos
//----------------------------------------------------------------------
localparam CLKS_PER_BIT = CLK_FREQ / BAUD_RATE;

//----------------------------------------------------------------------
// Definição dos estados da máquina de estados finita (FSM)
// FSM controla as etapas de transmissão: IDLE, START, DATA, STOP
//----------------------------------------------------------------------
localparam [2:0] IDLE      = 3'b000; // Esperando iniciar transmissão
localparam [2:0] START_BIT = 3'b001; // Transmitindo bit de início (start bit)
localparam [2:0] DATA_BITS = 3'b010; // Transmitindo os bits de dados
localparam [2:0] STOP_BIT  = 3'b011; // Transmitindo bit de parada (stop bit)

//----------------------------------------------------------------------
// Registradores internos que guardam o estado atual e dados de controle
//----------------------------------------------------------------------
reg [2:0]   state_reg, state_next;        // Estado atual e próximo da FSM
reg [15:0]  clk_count_reg, clk_count_next;// Conta os ciclos de clock para sincronizar os bits
reg [3:0]   bit_index_reg, bit_index_next;// Índice do bit atual que está sendo transmitido (0 a 7)
reg [7:0]   tx_data_reg, tx_data_next;    // Armazena localmente os dados a serem enviados

//----------------------------------------------------------------------
// BLOCO SEQUENCIAL: Atualiza os registradores a cada borda de subida do clock
//----------------------------------------------------------------------
always @(posedge i_clk or negedge i_rst_n)
begin
    if (!i_rst_n) begin
        // Se reset for ativado, volta tudo ao estado inicial
        state_reg       <= IDLE;
        clk_count_reg   <= 16'd0;
        bit_index_reg   <= 4'd0;
        tx_data_reg     <= 8'd0;
        o_tx_busy       <= 1'b0; // Não está transmitindo
    end else begin
        // Atualiza os valores com os "next" calculados pela lógica combinacional
        state_reg       <= state_next;
        clk_count_reg   <= clk_count_next;
        bit_index_reg   <= bit_index_next;
        tx_data_reg     <= tx_data_next;
        o_tx_busy       <= state_next != IDLE; // Ocupado quando não estiver em IDLE
    end
end

//----------------------------------------------------------------------
// BLOCO COMBINACIONAL: Lógica da máquina de estados (FSM)
// Define o próximo estado e valores dos registradores com base no estado atual
//----------------------------------------------------------------------
always @(*)
begin
    // Valores padrão (evita latches)
    state_next      = state_reg;
    clk_count_next  = clk_count_reg;
    bit_index_next  = bit_index_reg;
    tx_data_next    = tx_data_reg;

    case (state_reg)
        IDLE:
        begin
            // Estado inicial, esperando o pulso de início (i_tx_start)
            clk_count_next = 16'd0;
            bit_index_next = 4'd0;
            if (i_tx_start) begin
                // Se pulso recebido, começa a transmissão
                state_next      = START_BIT;
                tx_data_next    = i_tx_data; // Salva os dados localmente
                clk_count_next  = 16'd0;
            end
        end

        START_BIT:
        begin
            // Transmite o start bit (sempre 0) por CLKS_PER_BIT ciclos
            if (clk_count_reg == CLKS_PER_BIT - 1) begin
                state_next      = DATA_BITS; // Vai para próximo estado
                clk_count_next  = 16'd0;
                bit_index_next  = 4'd0;      // Começa pelo bit 0
            end else begin
                clk_count_next = clk_count_reg + 1; // Continua contando clock
            end
        end

        DATA_BITS:
        begin
            // Transmitindo os bits do dado, um por um
            if (clk_count_reg == CLKS_PER_BIT - 1) begin
                clk_count_next = 16'd0;
                if (bit_index_reg == 4'd7) begin
                    // Último bit foi enviado, vai pro stop bit
                    state_next = STOP_BIT;
                end else begin
                    // Vai pro próximo bit
                    bit_index_next = bit_index_reg + 1;
                end
            end else begin
                clk_count_next = clk_count_reg + 1;
            end
        end

        STOP_BIT:
        begin
            // Envia o stop bit (sempre 1) por CLKS_PER_BIT ciclos
            if (clk_count_reg == CLKS_PER_BIT - 1) begin
                state_next      = IDLE;     // Finaliza transmissão
                clk_count_next  = 16'd0;
            end else begin
                clk_count_next = clk_count_reg + 1;
            end
        end

        default:
        begin
            // Estado inválido, retorna para IDLE
            state_next = IDLE;
        end
    endcase
end

//----------------------------------------------------------------------
// BLOCO DE SAÍDA SERIAL: Define o valor da linha de transmissão UART (TX)
// UART é "nível alto" quando ociosa, envia start (0), depois 8 bits de dados,
// depois stop (1)
//----------------------------------------------------------------------
always @(*)
begin
    case(state_reg)
        IDLE:       o_tx_serial = 1'b1;                        // Linha TX fica em nível alto (1) quando não transmite
        START_BIT:  o_tx_serial = 1'b0;                        // Start bit (sempre 0)
        DATA_BITS:  o_tx_serial = tx_data_reg[bit_index_reg];  // Transmite o bit correspondente do dado
        STOP_BIT:   o_tx_serial = 1'b1;                        // Stop bit (sempre 1)
        default:    o_tx_serial = 1'b1;                        // Segurança: mantém em nível alto
    endcase
end

endmodule


module Entrada (
    input wire          Clock,     // Clock principal do sistema (50 MHz)
    input wire  [7:0]   GPIO_IN,   // Vetor 8 bits (entrada) 
    output reg  [16:0]  Dado,      // Saída do módulo 
    output wire         UART_TXD   // Saída serial UART (linha TX)
);

//----------------------------------------------------------------------
// Parâmetros fixos do sistema
//----------------------------------------------------------------------

localparam CLK_FREQ     = 50_000_000; // Frequência do clock: 50 MHz
localparam BAUD_RATE    = 9600;       // Baud rate da UART: 9600 bps

//----------------------------------------------------------------------
// Sinais internos usados para a comunicação UART
//----------------------------------------------------------------------

reg [7:0]   char_to_send;    // Byte a ser enviado via UART (caractere)
reg         tx_start_pulse; // Pulso que sinaliza início da transmissão UART
wire        uart_busy;      // Sinal que indica se o transmissor está ocupado

//----------------------------------------------------------------------
// Lógica de mapeamento dos GPIO_INitches para caracteres UART
// Essa lógica escolhe um caractere com base nas 5 primeiras chaves (GPIO_IN[4:0])
//----------------------------------------------------------------------

always @(*) begin
    casez (GPIO_IN[7:0]) // Define ações do robô com base na entrada
        8'b00000001: char_to_send = 8'd1; // Frente
        8'b00000010: char_to_send = 8'd2; // Trás
        8'b00000011: char_to_send = 8'd3; // Esquerda 
        8'b00000100: char_to_send = 8'd4; // Direita
		  8'b00000101: char_to_send = 8'd5; // + velocidade
		  8'b00000110: char_to_send = 8'd6; // - velocidade
        8'b00000111: char_to_send = 8'd7; // Giro
		  8'b00001000: char_to_send = 8'd8; // Parar
    endcase
end

//----------------------------------------------------------------------
// Geração de pulso para iniciar a transmissão UART
// Sempre que o transmissor não estiver ocupado, gera um pulso de 1 ciclo
// para iniciar o envio do dado atual
//----------------------------------------------------------------------

always @(posedge Clock) begin
    if (!uart_busy) begin
        tx_start_pulse <= 1'b1; // Começa a transmissão
    end else begin
        tx_start_pulse <= 1'b0; // Espera até que o UART esteja livre
    end
end

//----------------------------------------------------------------------
// Instanciação do módulo UART transmissor
//----------------------------------------------------------------------

uart_tx #(
    .CLK_FREQ(CLK_FREQ),
    .BAUD_RATE(BAUD_RATE)
) uart_transmitter (
    .i_clk(Clock),              // Clock do sistema
    .i_rst_n(1'b1),             // Reset desativado (ativo em 0) 
    .i_tx_start(tx_start_pulse),// Pulso que inicia a transmissão
    .i_tx_data(char_to_send),   // Dado que será transmitido (8 bits)
    .o_tx_serial(UART_TXD),     // Linha de saída serial → vai para o módulo Bluetooth, por exemplo
    .o_tx_busy(uart_busy)       // Indica se o transmissor está ocupado
);

//----------------------------------------------------------------------
// Atribuição do vetor Dado com os valores atuais das chaves GPIO_IN
//----------------------------------------------------------------------

always @(*) begin
    Dado[7]  = GPIO_IN[7];
    Dado[6]  = GPIO_IN[6];
    Dado[5]  = GPIO_IN[5];
    Dado[4]  = GPIO_IN[4];
    Dado[3]  = GPIO_IN[3];
    Dado[2]  = GPIO_IN[2];
    Dado[1]  = GPIO_IN[1];
    Dado[0]  = GPIO_IN[0];
end

endmodule




module MemoriaDeInstrucoes (Clock, Endereco, Instrucao);
	input wire Clock;
	input wire [31:0] Endereco;
	output reg [31:0] Instrucao;
	
	reg [31:0] Memoria[2048 * 11:0]; 
	
	always @ (posedge Clock)
		begin		

/***************** SO *****************/

Memoria[0] = {6'b010101, 26'd42}; // jumpMain - SO

/*** Proc 1 ***/
Memoria[1] = {6'b000001, 5'd0, 5'd30, 16'd3072}; // addi - Store do processo
Memoria[2] = {6'b010101, 26'd100}; // Pula para o Store

Memoria[3] = {6'b000001, 5'd0, 5'd30, 16'd3072}; // addi - Load do Processo
Memoria[4] = {6'b010101, 26'd150}; // Pula para o Load 

/*** Proc 2 ***/
Memoria[5] = {6'b000001, 5'd0, 5'd30, 16'd5120}; // addi - Store do processo
Memoria[6] = {6'b010101, 26'd100}; // Pula para o Store
 
Memoria[7] = {6'b000001, 5'd0, 5'd30, 16'd5120}; // addi - Load do Processo
Memoria[8] = {6'b010101, 26'd150}; // Pula para o Load 

/*** Proc 3 ***/
Memoria[9] = {6'b000001, 5'd0, 5'd30, 16'd7168}; // addi - Store do processo
Memoria[10] = {6'b010101, 26'd100}; // Pula para o Store
 
Memoria[11] = {6'b000001, 5'd0, 5'd30, 16'd7168}; // addi - Load do Processo
Memoria[12] = {6'b010101, 26'd150}; // Pula para o Load 

/*** Proc 4 ***/
Memoria[13] = {6'b000001, 5'd0, 5'd30, 16'd9216}; // addi - Store do processo
Memoria[14] = {6'b010101, 26'd100}; // Pula para o Store
 
Memoria[15] = {6'b000001, 5'd0, 5'd30, 16'd9216}; // addi - Load do Processo
Memoria[16] = {6'b010101, 26'd150}; // Pula para o Load 

/*** Proc 5 ***/
Memoria[17] = {6'b000001, 5'd0, 5'd30, 16'd11264}; // addi - Store do processo
Memoria[18] = {6'b010101, 26'd100}; // Pula para o Store
 
Memoria[19] = {6'b000001, 5'd0, 5'd30, 16'd11264}; // addi - Load do Processo
Memoria[20] = {6'b010101, 26'd150}; // Pula para o Load 

/*** Proc 6 ***/
Memoria[21] = {6'b000001, 5'd0, 5'd30, 16'd13312}; // addi - Store do processo
Memoria[22] = {6'b010101, 26'd100}; // Pula para o Store
 
Memoria[23] = {6'b000001, 5'd0, 5'd30, 16'd13312}; // addi - Load do Processo
Memoria[24] = {6'b010101, 26'd150}; // Pula para o Load 

/*** Proc 7 ***/
Memoria[25] = {6'b000001, 5'd0, 5'd30, 16'd15360}; // addi - Store do processo
Memoria[26] = {6'b010101, 26'd100}; // Pula para o Store
 
Memoria[27] = {6'b000001, 5'd0, 5'd30, 16'd15360}; // addi - Load do Processo
Memoria[28] = {6'b010101, 26'd150}; // Pula para o Load 

/*** Proc 8 ***/
Memoria[29] = {6'b000001, 5'd0, 5'd30, 16'd17408}; // addi - Store do processo
Memoria[30] = {6'b010101, 26'd100}; // Pula para o Store
 
Memoria[31] = {6'b000001, 5'd0, 5'd30, 16'd17408}; // addi - Load do Processo
Memoria[32] = {6'b010101, 26'd150}; // Pula para o Load 

/*** Proc 9 ***/
Memoria[33] = {6'b000001, 5'd0, 5'd30, 16'd19456}; // addi - Store do processo
Memoria[34] = {6'b010101, 26'd100}; // Pula para o Store
 
Memoria[35] = {6'b000001, 5'd0, 5'd30, 16'd19456}; // addi - Load do Processo
Memoria[36] = {6'b010101, 26'd150}; // Pula para o Load 

/*** Proc 10 ***/
Memoria[37] = {6'b000001, 5'd0, 5'd30, 16'd21504}; // addi - Store do processo
Memoria[38] = {6'b010101, 26'd100}; // Pula para o Store
 
Memoria[39] = {6'b000001, 5'd0, 5'd30, 16'd21504}; // addi - Load do Processo
Memoria[40] = {6'b010101, 26'd150}; // Pula para o Load 

Memoria[42] = {6'b010101, 26'd43}; // 
// Memoria[43] = {6'b011000, 5'd7, 21'd0}; // input
Memoria[43] = {6'b000001, 5'd0, 5'd7, 16'd1}; // addi - numero 1 para configurar sem preempção


Memoria[44] = {6'b001010, 5'd0, 5'd7, 16'd0}; // store
Memoria[45] = {6'b001000, 5'd0, 5'd8, 16'd0}; // load
Memoria[46] = {6'b000001, 5'd0, 5'd9, 16'd0}; // addi
Memoria[47] = {6'b010010, 5'd8, 5'd9, 16'd50}; // bneq 
Memoria[48] = {6'b010101, 26'd99}; // se for igual a zero - com preempção

// Memoria[50] = {6'b011000, 5'd7, 21'd0}; // input - não é igual a zero - Escolhe um programa 
Memoria[50] = {6'b000001, 5'd0, 5'd7, 16'd1}; // input - não é igual a zero - Escolhe um programa 1
Memoria[51] = {6'b001010, 5'd0, 5'd7, 16'd0}; // store
Memoria[52] = {6'b001000, 5'd0, 5'd8, 16'd0}; // load
Memoria[53] = {6'b000001, 5'd0, 5'd9, 16'd1}; // addi - programa 1
Memoria[54] = {6'b010010, 5'd8, 5'd9, 16'd56}; // bneq - não é o prog1 : pula
Memoria[55] = {6'b010101, 26'd2065}; 

Memoria[56] = {6'b000001, 5'd0, 5'd9, 16'd2}; // addi - programa 1
Memoria[57] = {6'b010010, 5'd8, 5'd9, 16'd99}; // bneq - não é o prog1 : pula
Memoria[58] = {6'b010101, 26'd4113};

Memoria[99] = {6'b111111, 26'b0}; // Alternar para Proc

// if numprocesso == 1 jump to 99 e etc...

/* Store - Guardar infos proc */
Memoria[100] = {6'b001010, 5'd30,  5'd0, 16'd0}; // store
Memoria[101] = {6'b001010, 5'd30,  5'd1, 16'd1}; // store
Memoria[102] = {6'b001010, 5'd30,  5'd2, 16'd2}; // store
Memoria[103] = {6'b001010, 5'd30,  5'd3, 16'd3}; // store
Memoria[104] = {6'b001010, 5'd30,  5'd4, 16'd4}; // store
Memoria[105] = {6'b001010, 5'd30,  5'd5, 16'd5}; // store
Memoria[106] = {6'b001010, 5'd30,  5'd6, 16'd6}; // store
Memoria[107] = {6'b001010, 5'd30,  5'd7, 16'd7}; // store
Memoria[108] = {6'b001010, 5'd30,  5'd8, 16'd8}; // store
Memoria[109] = {6'b001010, 5'd30,  5'd9, 16'd9}; // store
Memoria[110] = {6'b001010, 5'd30,  5'd10, 16'd10}; // store
Memoria[111] = {6'b001010, 5'd30,  5'd11, 16'd11}; // store
Memoria[112] = {6'b001010, 5'd30,  5'd12, 16'd12}; // store
Memoria[113] = {6'b001010, 5'd30,  5'd13, 16'd13}; // store
Memoria[114] = {6'b001010, 5'd30,  5'd14, 16'd14}; // store
Memoria[115] = {6'b001010, 5'd30,  5'd15, 16'd15}; // store
Memoria[116] = {6'b001010, 5'd30,  5'd16, 16'd16}; // store
Memoria[117] = {6'b001010, 5'd30,  5'd17, 16'd17}; // store
Memoria[118] = {6'b001010, 5'd30,  5'd18, 16'd18}; // store
Memoria[119] = {6'b001010, 5'd30,  5'd19, 16'd19}; // store
Memoria[120] = {6'b001010, 5'd30,  5'd20, 16'd20}; // store
Memoria[121] = {6'b001010, 5'd30,  5'd21, 16'd21}; // store
Memoria[122] = {6'b001010, 5'd30,  5'd22, 16'd22}; // store
Memoria[123] = {6'b001010, 5'd30,  5'd23, 16'd23}; // store
Memoria[124] = {6'b001010, 5'd30,  5'd24, 16'd24}; // store
Memoria[125] = {6'b001010, 5'd30,  5'd25, 16'd25}; // store
Memoria[126] = {6'b001010, 5'd30,  5'd26, 16'd26}; // store
Memoria[127] = {6'b001010, 5'd30,  5'd27, 16'd27}; // store
Memoria[128] = {6'b001010, 5'd30,  5'd28, 16'd28}; // store
Memoria[129] = {6'b001010, 5'd30,  5'd29, 16'd29}; // store
Memoria[130] = {6'b001010, 5'd30,  5'd30, 16'd30}; // store
Memoria[131] = {6'b001010, 5'd30,  5'd31, 16'd31}; // store
Memoria[132] = {6'b010101, 26'd99}; // jumpMain - SO

/* Load SO */
Memoria[150] = {6'b001000, 5'd30, 5'd0, 16'd0}; // load
Memoria[151] = {6'b001000, 5'd30, 5'd1, 16'd1}; // load
Memoria[152] = {6'b001000, 5'd30, 5'd2, 16'd2}; // load
Memoria[153] = {6'b001000, 5'd30, 5'd3, 16'd3}; // load
Memoria[154] = {6'b001000, 5'd30, 5'd4, 16'd4}; // load
Memoria[155] = {6'b001000, 5'd30, 5'd5, 16'd5}; // load
Memoria[156] = {6'b001000, 5'd30, 5'd6, 16'd6}; // load
Memoria[157] = {6'b001000, 5'd30, 5'd7, 16'd7}; // load
Memoria[158] = {6'b001000, 5'd30, 5'd8, 16'd8}; // load
Memoria[159] = {6'b001000, 5'd30, 5'd9, 16'd9}; // load
Memoria[160] = {6'b001000, 5'd30, 5'd10, 16'd10}; // load
Memoria[161] = {6'b001000, 5'd30, 5'd11, 16'd11}; // load
Memoria[162] = {6'b001000, 5'd30, 5'd12, 16'd12}; // load
Memoria[163] = {6'b001000, 5'd30, 5'd13, 16'd13}; // load
Memoria[164] = {6'b001000, 5'd30, 5'd14, 16'd14}; // load
Memoria[165] = {6'b001000, 5'd30, 5'd15, 16'd15}; // load
Memoria[166] = {6'b001000, 5'd30, 5'd16, 16'd16}; // load
Memoria[167] = {6'b001000, 5'd30, 5'd17, 16'd17}; // load
Memoria[168] = {6'b001000, 5'd30, 5'd18, 16'd18}; // load
Memoria[169] = {6'b001000, 5'd30, 5'd19, 16'd19}; // load
Memoria[170] = {6'b001000, 5'd30, 5'd20, 16'd20}; // load
Memoria[171] = {6'b001000, 5'd30, 5'd21, 16'd21}; // load
Memoria[172] = {6'b001000, 5'd30, 5'd22, 16'd22}; // load
Memoria[173] = {6'b001000, 5'd30, 5'd23, 16'd23}; // load
Memoria[174] = {6'b001000, 5'd30, 5'd24, 16'd24}; // load
Memoria[175] = {6'b001000, 5'd30, 5'd25, 16'd25}; // load
Memoria[176] = {6'b001000, 5'd30, 5'd26, 16'd26}; // load
Memoria[177] = {6'b001000, 5'd30, 5'd27, 16'd27}; // load
Memoria[178] = {6'b001000, 5'd30, 5'd28, 16'd28}; // load
Memoria[179] = {6'b001000, 5'd30, 5'd29, 16'd29}; // load
Memoria[180] = {6'b001000, 5'd30, 5'd30, 16'd30}; // load
Memoria[181] = {6'b001000, 5'd30, 5'd31, 16'd31}; // load
Memoria[182] = {6'b010101, 26'd99}; // jumpMain - SO

/***************** Programa 1 *****************/
//// Com preempção
//Memoria[2048] = {6'b000001, 5'd0, 5'd29, 16'd2048}; // addi - Load do Processo
//Memoria[2049] = {6'b000001, 5'd0, 5'd28, 16'd500}; // addi
//Memoria[2050] = {6'b010101, 26'd2051}; // jumpMain
//Memoria[2051] = {6'b001010, 5'd29, 5'd0, 16'd0}; // store
//Memoria[2052] = {6'b001010, 5'd29, 5'd0, 16'd1}; // store
//Memoria[2053] = {6'b001010, 5'd29, 5'd0, 16'd2}; // store
////Memoria[2054] = {6'b000001, 5'd0, 5'd8, 16'd10000}; // addi
//Memoria[2054] = {6'b011000, 5'd8, 21'd0}; // input
//Memoria[2055] = {6'b001010, 5'd29, 5'd8, 16'd0}; // store
//Memoria[2056] = {6'b000001, 5'd0, 5'd8, 16'd1000}; // addi
//Memoria[2057] = {6'b001010, 5'd29, 5'd8, 16'd1}; // store
//Memoria[2058] = {6'b001000, 5'd29, 5'd8, 16'd0}; // load
//Memoria[2059] = {6'b001000, 5'd29, 5'd9, 16'd1}; // load
//Memoria[2060] = {6'b000000, 5'd8, 5'd9, 5'd10, 11'd0}; // add
//Memoria[2061] = {6'b001010, 5'd29, 5'd10, 16'd10}; // store
//Memoria[2062] = {6'b001000, 5'd29, 5'd10, 16'd10}; // load
//Memoria[2063] = {6'b011001, 5'd10, 5'd10, 16'd0}; // output
//Memoria[2064] = {6'b111111, 5'd0, 21'd0}; // end main

/// Sem preempção
Memoria[2065] = {6'b011000, 5'd8, 21'd0}; // input
Memoria[2066] = {6'b010101, 26'd2065}; ; // jump inicio do programa para ler nova entrada
//
//
///***************** Programa 2 *****************/
//Memoria[4096] = {6'b000001, 5'd0, 5'd29, 16'd4096}; // addi - Load do Processo
//Memoria[4097] = {6'b000001, 5'd0, 5'd28, 16'd500}; // addi
//Memoria[4098] = {6'b010101, 26'd4099}; // jumpMain
//Memoria[4099] = {6'b001010, 5'd29, 5'd0, 16'd0}; // store
//Memoria[4100] = {6'b001010, 5'd29, 5'd0, 16'd1}; // store
//Memoria[4101] = {6'b001010, 5'd29, 5'd0, 16'd2}; // store
//Memoria[4102] = {6'b000001, 5'd0, 5'd8, 16'd8000}; // addi
//Memoria[4103] = {6'b001010, 5'd29, 5'd8, 16'd0}; // store
////Memoria[4104] = {6'b000001, 5'd0, 5'd8, 16'd5000}; // addi
//Memoria[4104] = {6'b011000, 5'd8, 21'd0}; // input
//Memoria[4105] = {6'b001010, 5'd29, 5'd8, 16'd1}; // store
//Memoria[4106] = {6'b001000, 5'd29, 5'd8, 16'd0}; // load
//Memoria[4107] = {6'b001000, 5'd29, 5'd9, 16'd1}; // load
//Memoria[4108] = {6'b000010, 5'd8, 5'd9, 5'd10, 11'd0}; // sub
//Memoria[4109] = {6'b001010, 5'd29, 5'd10, 16'd10}; // store
//Memoria[4110] = {6'b001000, 5'd29, 5'd10, 16'd10}; // load
//Memoria[4111] = {6'b011001, 5'd10, 5'd10, 16'd0}; // output
//Memoria[4112] = {6'b111111, 5'd0, 21'd0}; // end main
//
//// sem preempção
//Memoria[4113] = {6'b000001, 5'd0, 5'd29, 16'd4096}; // addi - Load do Processo
//Memoria[4114] = {6'b000001, 5'd0, 5'd28, 16'd500}; // addi
//Memoria[4115] = {6'b010101, 26'd4116}; // jumpMain
//Memoria[4116] = {6'b001010, 5'd29, 5'd0, 16'd0}; // store
//Memoria[4117] = {6'b001010, 5'd29, 5'd0, 16'd1}; // store
//Memoria[4118] = {6'b001010, 5'd29, 5'd0, 16'd2}; // store
//Memoria[4119] = {6'b000001, 5'd0, 5'd8, 16'd8000}; // addi
//Memoria[4120] = {6'b001010, 5'd29, 5'd8, 16'd0}; // store
////Memoria[4121] = {6'b000001, 5'd0, 5'd8, 16'd5000}; // addi
//Memoria[4121] = {6'b011000, 5'd8, 21'd0}; // input
//Memoria[4122] = {6'b001010, 5'd29, 5'd8, 16'd1}; // store
//Memoria[4123] = {6'b001000, 5'd29, 5'd8, 16'd0}; // load
//Memoria[4124] = {6'b001000, 5'd29, 5'd9, 16'd1}; // load
//Memoria[4125] = {6'b000010, 5'd8, 5'd9, 5'd10, 11'd0}; // sub
//Memoria[4126] = {6'b001010, 5'd29, 5'd10, 16'd10}; // store
//Memoria[4127] = {6'b001000, 5'd29, 5'd10, 16'd10}; // load
//Memoria[4128] = {6'b011001, 5'd10, 5'd10, 16'd0}; // output
//Memoria[4129] = {6'b111111, 5'd0, 21'd0}; // end main
//Memoria[4130] = {6'b101111, 26'd0}; // mensagem
//Memoria[4131] = {6'b010101, 26'd22000}; // jump end main
//
//Memoria[6144] = {6'b111111, 5'd0, 21'd0}; // end main
//
//Memoria[8192] = {6'b111111, 5'd0, 21'd0}; // end main
//
//Memoria[10240] = {6'b111111, 5'd0, 21'd0}; // end mai
//
//Memoria[12288] = {6'b111111, 5'd0, 21'd0}; // end main
//
//Memoria[14336] = {6'b111111, 5'd0, 21'd0}; // end main
//
//Memoria[16384] = {6'b111111, 5'd0, 21'd0}; // end main
//
//Memoria[18432] = {6'b111111, 5'd0, 21'd0}; // end main
//
//Memoria[20480] = {6'b111111, 5'd0, 21'd0}; // end main

Memoria[22000] = {6'b010101, 26'd22000}; // end SO

		Instrucao = Memoria[Endereco];
		end
endmodule


module GerenciadorDeProcessos (clock, parar, entradaParaLCD, pcAntigo, chaveGPIO_IN,  FilaProcessos, NumProcesso, NumProcessoAnt, NumProxProcesso, Instrucao, Encerrar, contador, EnderecoPC, SemPreemp, infoMux4, execStore, execLoad); // Tem que colocar duas instruções de final de código 
	input clock;
	
	input chaveGPIO_IN;
	
	input [31:0] Instrucao, infoMux4;
	
	wire [5:0] Opcode;
	assign Opcode = Instrucao[31:26];
	
	output reg [1:0] FilaProcessos;
	output reg [3:0] NumProcesso, NumProcessoAnt, NumProxProcesso; 
	output reg Encerrar, SemPreemp, parar;
	output reg [4:0] contador;
	output reg [31:0] EnderecoPC, pcAntigo;
	output reg [15:0] entradaParaLCD;
	
	integer i;
	
	reg EstadoProc1, EstadoProc2, EstadoProc3, EstadoProc4, EstadoProc5, EstadoProc6, EstadoProc7, EstadoProc8, EstadoProc9, EstadoProc10;
	reg [31:0] StoreProc1, StoreProc2, StoreProc3, StoreProc4, StoreProc5, StoreProc6, StoreProc7, StoreProc8, StoreProc9, StoreProc10;
	reg [31:0] LoadProc1, LoadProc2, LoadProc3, LoadProc4, LoadProc5, LoadProc6, LoadProc7, LoadProc8, LoadProc9, LoadProc10;
	reg [31:0] PcProcSO, PcProcSOFim, PcProc1, PcProc2, PcProc3, PcProc4, PcProc5, PcProc6, PcProc7, PcProc8, PcProc9, PcProc10;
	output reg execStore, execLoad;
	
	initial
	begin
		EnderecoPC = 32'd0;
		NumProcesso = 4'b0;
		NumProcessoAnt = 4'd15;
		NumProxProcesso = 4'b0;
		
		SemPreemp = 1'b0;
		
		EstadoProc1 = 1'b0;
		EstadoProc2 = 1'b0;
		
		Encerrar = 1'b0;
		contador = 5'd0;
		 
		PcProcSO = 32'd0;
		PcProcSOFim = 32'd22000; // definir
		PcProc1 = 32'd2048;
		PcProc2 = 32'd4096;
		
		StoreProc1 = 32'd1;
		StoreProc2 = 32'd5;
		
		LoadProc1 = 32'd3;
		LoadProc2 = 32'd7;
		
		execLoad = 1'b0;
		execStore = 1'b0;
    end

	// Todos os processos começam com 0
	// Conforme os processos forem terminando, as posições vão sendo preenchidas com 1
	// Busca o primeiro processo pronto para ser executado
	always @ (negedge clock) 
		begin
		
        //entrada
        if (Opcode == 6'b101111) begin entradaParaLCD = 16'd15; EnderecoPC = 31'd22000; end
        else begin
            if (Opcode == 6'b011000 && parar == 1'b0) begin parar = 1'b1; entradaParaLCD = 16'd11; end
    		if (Opcode == 6'b011000 && parar == 1'b1 && chaveGPIO_IN == 1'b0) begin EnderecoPC = pcAntigo; entradaParaLCD = 16'd11; end 
            else if(Opcode == 6'b011000 && parar == 1'b1 && chaveGPIO_IN == 1'b1) begin
    			parar = 1'b0;
    			EnderecoPC = infoMux4;
    			entradaParaLCD = 16'd16;
    		end
    		
    	    //saida
    	    if (Opcode == 6'b011001 && parar == 1'b0) begin parar = 1'b1; entradaParaLCD = 16'd13; end
    		if (Opcode == 6'b011001 && parar == 1'b1 && chaveGPIO_IN == 1'b0) begin EnderecoPC = pcAntigo; entradaParaLCD = 16'd13; end 
    		else if(Opcode == 6'b011001 && parar == 1'b1 && chaveGPIO_IN == 1'b1) begin
    			parar = 1'b0;
    			EnderecoPC = infoMux4;
    			entradaParaLCD = 16'd16;
    		end
    			
    		else if (parar == 1'b0) begin // não é para travar
    			if (FilaProcessos[0] == 1'b1 && FilaProcessos[1] == 1'b1 )
    				Encerrar = 1'b1;
    				
    			if (Encerrar == 1'b1) begin
    			    entradaParaLCD = 16'd14;
    				EnderecoPC = PcProcSOFim;
    			end // fim if encerrar
    			
    			
    			else begin 
    				if (NumProcesso == 4'd0 && Opcode != 6'b111111 && execStore == 1'b0 && execLoad == 1'b0) begin SemPreemp = 1'b1; EnderecoPC = infoMux4; entradaParaLCD = 16'd15; end // Roda SO
    				
    				else if(NumProcesso == 4'd0 && NumProcessoAnt == 4'd15 && Opcode == 6'b111111) begin // Primeiro processo, não precisa fazer o load, só vai para o primeiro proc disponível 
    						SemPreemp = 1'b0; 
    						if (FilaProcessos[0] == 1'b0) begin EnderecoPC = PcProc1; NumProcessoAnt = 4'd0; NumProcesso = 4'd1; entradaParaLCD = 16'd1; end
    						else if (FilaProcessos[1] == 1'b0) begin EnderecoPC = PcProc2; NumProcessoAnt = 4'd0; NumProcesso = 4'd2; entradaParaLCD = 16'd2; end
    				end
    				
    				else if (NumProcesso == 4'd0 && execStore == 1'b1 && execLoad == 1'b1) begin// STORE
    				    entradaParaLCD = 16'd12;
    					SemPreemp = 1'b1; execLoad = 1'b0; contador = 5'b0;
    					
    					// escolhe endereço pra fazer o store
    					if (NumProcessoAnt == 4'd1) begin EnderecoPC = StoreProc1; end
    					else if (NumProcessoAnt == 4'd2) begin EnderecoPC = StoreProc2; end
    					
    				end
    				
    				else if (NumProcesso == 4'd0 && Opcode != 6'b111111 && execStore == 1'b1 && execLoad == 1'b0) begin // STORE
    				    entradaParaLCD = 16'd12;
    					SemPreemp = 1'b1; EnderecoPC = infoMux4;
    				end
    				
    				else if (NumProcesso == 4'd0 && Opcode == 6'b111111 && execStore == 1'b1 && execLoad == 1'b0) begin // ACABOU STORE - ESCOLHE PROC E FAZ LOAD 
    					entradaParaLCD = 16'd12;
    					SemPreemp = 1'b1; execStore = 1'b0; execLoad = 1'b1;
    				
    					// Trocou o contexto, decide novo processo
    					if (FilaProcessos[0] == 1'b0 && NumProcessoAnt != 4'd1) begin EnderecoPC = LoadProc1; NumProxProcesso = 4'd1;end
    					else if (FilaProcessos[1] == 1'b0 && NumProcessoAnt != 4'd2) begin EnderecoPC = LoadProc2; NumProxProcesso = 4'd2;end
    				end
    				
    				else if (NumProcesso == 4'd0 && execStore == 1'b0 && execLoad == 1'b1 && Opcode != 6'b111111) begin // FAZENDO LOAD
    						entradaParaLCD = 16'd12;
    						SemPreemp = 1'b1; EnderecoPC = infoMux4;
    				end
    				
    				else if (NumProcesso == 4'd0 && execStore == 1'b0 && execLoad == 1'b1 && Opcode == 6'b111111) begin // ACABOU LOAD - Continua PC processo
    						execLoad = 1'b0; SemPreemp = 1'b0; NumProcesso = NumProxProcesso; NumProcessoAnt = 4'b0;
    						
    						case(NumProcesso)
    							4'd1: begin EnderecoPC = PcProc1; entradaParaLCD = 16'd1; end
    							4'd2: begin EnderecoPC = PcProc2; entradaParaLCD = 16'd2; end
    						endcase
    				end
    				
    				else if (NumProcesso != 4'd0 && Opcode == 6'b111111) begin
    					contador = 5'b11111;
    					case (NumProcesso) 
    						4'd1: begin EstadoProc1 = 1'b1; entradaParaLCD = 16'd1; end 
    						4'd2: begin EstadoProc2 = 1'b1; entradaParaLCD = 16'd2; end
    					 endcase
    
    					// Atualiza a fila de processos
    					FilaProcessos[0] = (EstadoProc1 == 1'b1) ? 1'b1 : 1'b0;
    					FilaProcessos[1] = (EstadoProc2 == 1'b1) ? 1'b1 : 1'b0;
    				end
    				
    				else if(NumProcesso != 4'd0 && Opcode != 6'b111111) begin
    					if (NumProcesso == 4'd1) begin PcProc1 = infoMux4; EnderecoPC = infoMux4; entradaParaLCD = 16'd1; end
    					else if (NumProcesso == 4'd2) begin PcProc2 = infoMux4; EnderecoPC = infoMux4; entradaParaLCD = 16'd2; end
    				end
    				
    				// Preempção
    				if (contador < 5'd12) begin 
    					 if (SemPreemp == 1'b0) begin contador = contador + 1'd1; end
    					 
    				end
    				else begin
    					contador = 5'd0;
    					SemPreemp = 1'b1; 
    					execStore = 1'b1;
    					execLoad = 1'b1;
    				
    					NumProcessoAnt = NumProcesso; NumProcesso = 4'd0; EnderecoPC = PcProcSO;
    				end
    			end
    			pcAntigo = EnderecoPC;
    		end
    	end
	end // always*/
endmodule

 module ProgramCounter (Entrada, Saida);
	input wire [31:0] Entrada;
	output reg [31:0] Saida;
	
	always @ (Entrada)
		Saida = Entrada;
				
endmodule





module    Reset_Delay(iCLK,oRESET);
input        iCLK;
output reg    oRESET;
reg    [19:0]    Cont;

always@(posedge iCLK)
begin
    if(Cont!=20'hFFFFF)
    begin
        Cont    <=    Cont+1'b1;
        oRESET    <=    1'b0;
    end
    else
    oRESET    <=    1'b1;
end

endmodule




module LCD_display_string
(
	input [4:0] index,
	input [3:0] hex0,hex1,
	input [15:0] reg_msg,
	output reg [7:0] out
);
 always @ (reg_msg) begin
			case (reg_msg)
			16'd0: // Bem - vindo!
			begin
				case (index)
					5'h00: out <= 8'h20;
                    5'h01: out <= 8'h42;
                    5'h02: out <= 8'h65;
                    5'h03: out <= 8'h6D;
                    5'h04: out <= 8'h20;
                    5'h05: out <= 8'h2D;
                    5'h06: out <= 8'h20;
                    5'h07: out <= 8'h76;
                    5'h08: out <= 8'h69;
                    5'h09: out <= 8'h6E;
                    5'h0A: out <= 8'h64;
                    5'h0B: out <= 8'h6F;
                    5'h0C: out <= 8'h21;
				default: out <= 8'h20;
				endcase
			end
			16'd1: // Exeutando processo 1
			begin
			    case (index)
			        5'h00: out <= 8'h20;
                    5'h01: out <= 8'h45;
                    5'h02: out <= 8'h78;
                    5'h03: out <= 8'h65;
                    5'h04: out <= 8'h63;
                    5'h05: out <= 8'h75;
                    5'h06: out <= 8'h74;
                    5'h07: out <= 8'h61;
                    5'h08: out <= 8'h6E;
                    5'h09: out <= 8'h64;
                    5'h0A: out <= 8'h6F;
                    5'h0B: out <= 8'h20;
                    5'h0C: out <= 8'h20;
                    5'h0D: out <= 8'h20;
                    5'h0E: out <= 8'h20;
                    5'h0F: out <= 8'h20;
                    5'h10: out <= 8'h20;
                    5'h11: out <= 8'h20;
                    5'h12: out <= 8'h50;
                    5'h13: out <= 8'h72;
                    5'h14: out <= 8'h6F;
                    5'h15: out <= 8'h63;
                    5'h16: out <= 8'h65;
                    5'h17: out <= 8'h73;
                    5'h18: out <= 8'h73;
                    5'h19: out <= 8'h6F;
                    5'h1A: out <= 8'h20;
                    5'h1B: out <= 8'h20;
                    5'h1C: out <= 8'h31;
                    5'h1D: out <= 8'h2E;
                    5'h1E: out <= 8'h2E;
                    5'h1F: out <= 8'h2E;
	                default: out <= 8'h20;
				endcase
			end
			16'd2: // Exeutando processo 1
			begin
			    case (index)
			        5'h00: out <= 8'h20;
                    5'h01: out <= 8'h45;
                    5'h02: out <= 8'h78;
                    5'h03: out <= 8'h65;
                    5'h04: out <= 8'h63;
                    5'h05: out <= 8'h75;
                    5'h06: out <= 8'h74;
                    5'h07: out <= 8'h61;
                    5'h08: out <= 8'h6E;
                    5'h09: out <= 8'h64;
                    5'h0A: out <= 8'h6F;
                    5'h0B: out <= 8'h20;
                    5'h0C: out <= 8'h20;
                    5'h0D: out <= 8'h20;
                    5'h0E: out <= 8'h20;
                    5'h0F: out <= 8'h20;
                    5'h10: out <= 8'h20;
                    5'h11: out <= 8'h20;
                    5'h12: out <= 8'h50;
                    5'h13: out <= 8'h72;
                    5'h14: out <= 8'h6F;
                    5'h15: out <= 8'h63;
                    5'h16: out <= 8'h65;
                    5'h17: out <= 8'h73;
                    5'h18: out <= 8'h73;
                    5'h19: out <= 8'h6F;
                    5'h1A: out <= 8'h20;
                    5'h1B: out <= 8'h20;
                    5'h1C: out <= 8'h32;
                    5'h1D: out <= 8'h2E;
                    5'h1E: out <= 8'h2E;
                    5'h1F: out <= 8'h2E;
	                default: out <= 8'h20;
				endcase
			end
			16'd3: // Exeutando processo 1
			begin
			    case (index)
			        5'h00: out <= 8'h20;
                    5'h01: out <= 8'h45;
                    5'h02: out <= 8'h78;
                    5'h03: out <= 8'h65;
                    5'h04: out <= 8'h63;
                    5'h05: out <= 8'h75;
                    5'h06: out <= 8'h74;
                    5'h07: out <= 8'h61;
                    5'h08: out <= 8'h6E;
                    5'h09: out <= 8'h64;
                    5'h0A: out <= 8'h6F;
                    5'h0B: out <= 8'h20;
                    5'h0C: out <= 8'h20;
                    5'h0D: out <= 8'h20;
                    5'h0E: out <= 8'h20;
                    5'h0F: out <= 8'h20;
                    5'h10: out <= 8'h20;
                    5'h11: out <= 8'h20;
                    5'h12: out <= 8'h50;
                    5'h13: out <= 8'h72;
                    5'h14: out <= 8'h6F;
                    5'h15: out <= 8'h63;
                    5'h16: out <= 8'h65;
                    5'h17: out <= 8'h73;
                    5'h18: out <= 8'h73;
                    5'h19: out <= 8'h6F;
                    5'h1A: out <= 8'h20;
                    5'h1B: out <= 8'h20;
                    5'h1C: out <= 8'h33;
                    5'h1D: out <= 8'h2E;
                    5'h1E: out <= 8'h2E;
                    5'h1F: out <= 8'h2E;
	                default: out <= 8'h20;
				endcase
			end
			16'd4: // Exeutando processo 1
			begin
			    case (index)
			        5'h00: out <= 8'h20;
                    5'h01: out <= 8'h45;
                    5'h02: out <= 8'h78;
                    5'h03: out <= 8'h65;
                    5'h04: out <= 8'h63;
                    5'h05: out <= 8'h75;
                    5'h06: out <= 8'h74;
                    5'h07: out <= 8'h61;
                    5'h08: out <= 8'h6E;
                    5'h09: out <= 8'h64;
                    5'h0A: out <= 8'h6F;
                    5'h0B: out <= 8'h20;
                    5'h0C: out <= 8'h20;
                    5'h0D: out <= 8'h20;
                    5'h0E: out <= 8'h20;
                    5'h0F: out <= 8'h20;
                    5'h10: out <= 8'h20;
                    5'h11: out <= 8'h20;
                    5'h12: out <= 8'h50;
                    5'h13: out <= 8'h72;
                    5'h14: out <= 8'h6F;
                    5'h15: out <= 8'h63;
                    5'h16: out <= 8'h65;
                    5'h17: out <= 8'h73;
                    5'h18: out <= 8'h73;
                    5'h19: out <= 8'h6F;
                    5'h1A: out <= 8'h20;
                    5'h1B: out <= 8'h20;
                    5'h1C: out <= 8'h34;
                    5'h1D: out <= 8'h2E;
                    5'h1E: out <= 8'h2E;
                    5'h1F: out <= 8'h2E;
	                default: out <= 8'h20;
				endcase
			end
			16'd5: // Exeutando processo 1
			begin
			    case (index)
			        5'h00: out <= 8'h20;
                    5'h01: out <= 8'h45;
                    5'h02: out <= 8'h78;
                    5'h03: out <= 8'h65;
                    5'h04: out <= 8'h63;
                    5'h05: out <= 8'h75;
                    5'h06: out <= 8'h74;
                    5'h07: out <= 8'h61;
                    5'h08: out <= 8'h6E;
                    5'h09: out <= 8'h64;
                    5'h0A: out <= 8'h6F;
                    5'h0B: out <= 8'h20;
                    5'h0C: out <= 8'h20;
                    5'h0D: out <= 8'h20;
                    5'h0E: out <= 8'h20;
                    5'h0F: out <= 8'h20;
                    5'h10: out <= 8'h20;
                    5'h11: out <= 8'h20;
                    5'h12: out <= 8'h50;
                    5'h13: out <= 8'h72;
                    5'h14: out <= 8'h6F;
                    5'h15: out <= 8'h63;
                    5'h16: out <= 8'h65;
                    5'h17: out <= 8'h73;
                    5'h18: out <= 8'h73;
                    5'h19: out <= 8'h6F;
                    5'h1A: out <= 8'h20;
                    5'h1B: out <= 8'h20;
                    5'h1C: out <= 8'h35;
                    5'h1D: out <= 8'h2E;
                    5'h1E: out <= 8'h2E;
                    5'h1F: out <= 8'h2E;
	                default: out <= 8'h20;
				endcase
			end
			16'd6: // Exeutando processo 1
			begin
			    case (index)
			        5'h00: out <= 8'h20;
                    5'h01: out <= 8'h45;
                    5'h02: out <= 8'h78;
                    5'h03: out <= 8'h65;
                    5'h04: out <= 8'h63;
                    5'h05: out <= 8'h75;
                    5'h06: out <= 8'h74;
                    5'h07: out <= 8'h61;
                    5'h08: out <= 8'h6E;
                    5'h09: out <= 8'h64;
                    5'h0A: out <= 8'h6F;
                    5'h0B: out <= 8'h20;
                    5'h0C: out <= 8'h20;
                    5'h0D: out <= 8'h20;
                    5'h0E: out <= 8'h20;
                    5'h0F: out <= 8'h20;
                    5'h10: out <= 8'h20;
                    5'h11: out <= 8'h20;
                    5'h12: out <= 8'h50;
                    5'h13: out <= 8'h72;
                    5'h14: out <= 8'h6F;
                    5'h15: out <= 8'h63;
                    5'h16: out <= 8'h65;
                    5'h17: out <= 8'h73;
                    5'h18: out <= 8'h73;
                    5'h19: out <= 8'h6F;
                    5'h1A: out <= 8'h20;
                    5'h1B: out <= 8'h20;
                    5'h1C: out <= 8'h36;
                    5'h1D: out <= 8'h2E;
                    5'h1E: out <= 8'h2E;
                    5'h1F: out <= 8'h2E;
	                default: out <= 8'h20;
				endcase
			end
			16'd7: // Exeutando processo 1
			begin
			    case (index)
			        5'h00: out <= 8'h20;
                    5'h01: out <= 8'h45;
                    5'h02: out <= 8'h78;
                    5'h03: out <= 8'h65;
                    5'h04: out <= 8'h63;
                    5'h05: out <= 8'h75;
                    5'h06: out <= 8'h74;
                    5'h07: out <= 8'h61;
                    5'h08: out <= 8'h6E;
                    5'h09: out <= 8'h64;
                    5'h0A: out <= 8'h6F;
                    5'h0B: out <= 8'h20;
                    5'h0C: out <= 8'h20;
                    5'h0D: out <= 8'h20;
                    5'h0E: out <= 8'h20;
                    5'h0F: out <= 8'h20;
                    5'h10: out <= 8'h20;
                    5'h11: out <= 8'h20;
                    5'h12: out <= 8'h50;
                    5'h13: out <= 8'h72;
                    5'h14: out <= 8'h6F;
                    5'h15: out <= 8'h63;
                    5'h16: out <= 8'h65;
                    5'h17: out <= 8'h73;
                    5'h18: out <= 8'h73;
                    5'h19: out <= 8'h6F;
                    5'h1A: out <= 8'h20;
                    5'h1B: out <= 8'h20;
                    5'h1C: out <= 8'h37;
                    5'h1D: out <= 8'h2E;
                    5'h1E: out <= 8'h2E;
                    5'h1F: out <= 8'h2E;
	                default: out <= 8'h20;
				endcase
			end
			16'd8: // Exeutando processo 1
			begin
			    case (index)
			        5'h00: out <= 8'h20;
                    5'h01: out <= 8'h45;
                    5'h02: out <= 8'h78;
                    5'h03: out <= 8'h65;
                    5'h04: out <= 8'h63;
                    5'h05: out <= 8'h75;
                    5'h06: out <= 8'h74;
                    5'h07: out <= 8'h61;
                    5'h08: out <= 8'h6E;
                    5'h09: out <= 8'h64;
                    5'h0A: out <= 8'h6F;
                    5'h0B: out <= 8'h20;
                    5'h0C: out <= 8'h20;
                    5'h0D: out <= 8'h20;
                    5'h0E: out <= 8'h20;
                    5'h0F: out <= 8'h20;
                    5'h10: out <= 8'h20;
                    5'h11: out <= 8'h20;
                    5'h12: out <= 8'h50;
                    5'h13: out <= 8'h72;
                    5'h14: out <= 8'h6F;
                    5'h15: out <= 8'h63;
                    5'h16: out <= 8'h65;
                    5'h17: out <= 8'h73;
                    5'h18: out <= 8'h73;
                    5'h19: out <= 8'h6F;
                    5'h1A: out <= 8'h20;
                    5'h1B: out <= 8'h20;
                    5'h1C: out <= 8'h38;
                    5'h1D: out <= 8'h2E;
                    5'h1E: out <= 8'h2E;
                    5'h1F: out <= 8'h2E;
	                default: out <= 8'h20;
				endcase
			end
			16'd9: // Exeutando processo 1
			begin
			    case (index)
			        5'h00: out <= 8'h20;
                    5'h01: out <= 8'h45;
                    5'h02: out <= 8'h78;
                    5'h03: out <= 8'h65;
                    5'h04: out <= 8'h63;
                    5'h05: out <= 8'h75;
                    5'h06: out <= 8'h74;
                    5'h07: out <= 8'h61;
                    5'h08: out <= 8'h6E;
                    5'h09: out <= 8'h64;
                    5'h0A: out <= 8'h6F;
                    5'h0B: out <= 8'h20;
                    5'h0C: out <= 8'h20;
                    5'h0D: out <= 8'h20;
                    5'h0E: out <= 8'h20;
                    5'h0F: out <= 8'h20;
                    5'h10: out <= 8'h20;
                    5'h11: out <= 8'h20;
                    5'h12: out <= 8'h50;
                    5'h13: out <= 8'h72;
                    5'h14: out <= 8'h6F;
                    5'h15: out <= 8'h63;
                    5'h16: out <= 8'h65;
                    5'h17: out <= 8'h73;
                    5'h18: out <= 8'h73;
                    5'h19: out <= 8'h6F;
                    5'h1A: out <= 8'h20;
                    5'h1B: out <= 8'h20;
                    5'h1C: out <= 8'h39;
                    5'h1D: out <= 8'h2E;
                    5'h1E: out <= 8'h2E;
                    5'h1F: out <= 8'h2E;
	                default: out <= 8'h20;
				endcase
			end
			16'd10: // Exeutando processo 1
			begin
			    case (index)
			        5'h00: out <= 8'h20;
                    5'h01: out <= 8'h45;
                    5'h02: out <= 8'h78;
                    5'h03: out <= 8'h65;
                    5'h04: out <= 8'h63;
                    5'h05: out <= 8'h75;
                    5'h06: out <= 8'h74;
                    5'h07: out <= 8'h61;
                    5'h08: out <= 8'h6E;
                    5'h09: out <= 8'h64;
                    5'h0A: out <= 8'h6F;
                    5'h0B: out <= 8'h20;
                    5'h0C: out <= 8'h20;
                    5'h0D: out <= 8'h20;
                    5'h0E: out <= 8'h20;
                    5'h0F: out <= 8'h20;
                    5'h10: out <= 8'h20;
                    5'h11: out <= 8'h20;
                    5'h12: out <= 8'h50;
                    5'h13: out <= 8'h72;
                    5'h14: out <= 8'h6F;
                    5'h15: out <= 8'h63;
                    5'h16: out <= 8'h65;
                    5'h17: out <= 8'h73;
                    5'h18: out <= 8'h73;
                    5'h19: out <= 8'h6F;
                    5'h1A: out <= 8'h20;
                    5'h1B: out <= 8'h31;
                    5'h1C: out <= 8'h30;
                    5'h1D: out <= 8'h2E;
                    5'h1E: out <= 8'h2E;
                    5'h1F: out <= 8'h2E;
	                default: out <= 8'h20;
				endcase
			end
			16'd11: // Entrada uart
			begin
				case (index)
					5'h00: out <= 8'h45;
					5'h01: out <= 8'h6e;
					5'h02: out <= 8'h74;
					5'h03: out <= 8'h72;
					5'h04: out <= 8'h61;
					5'h05: out <= 8'h64;
					5'h06: out <= 8'h61;
					5'h07: out <= 8'h20; 
				    5'h08: out <= 8'h55; // 'U'
                    5'h09: out <= 8'h41; // 'A'
                    5'h0A: out <= 8'h52; // 'R'
                    5'h0B: out <= 8'h54; // 'T'
				default: out <= 8'h20;
				endcase
			end
			
			16'd12: //Trocando contexto... ok
			begin
				case (index)
					5'h00: out <= 8'h20;
                    5'h01: out <= 8'h54;
                    5'h02: out <= 8'h72;
                    5'h03: out <= 8'h6F;
                    5'h04: out <= 8'h63;
                    5'h05: out <= 8'h61;
                    5'h06: out <= 8'h6E;
                    5'h07: out <= 8'h64;
                    5'h08: out <= 8'h6F;
                    5'h09: out <= 8'h20;
                    5'h0A: out <= 8'h20;
                    5'h0B: out <= 8'h20;
                    5'h0C: out <= 8'h20;
                    5'h0D: out <= 8'h20;
                    5'h0E: out <= 8'h20;
                    5'h0F: out <= 8'h20;
                    5'h10: out <= 8'h20;
                    5'h11: out <= 8'h43;
                    5'h12: out <= 8'h6F;
                    5'h13: out <= 8'h6E;
                    5'h14: out <= 8'h74;
                    5'h15: out <= 8'h65;
                    5'h16: out <= 8'h78;
                    5'h17: out <= 8'h74;
                    5'h18: out <= 8'h6F;
                    5'h19: out <= 8'h20;
                    5'h1A: out <= 8'h20;
                    5'h1B: out <= 8'h20;
                    5'h1C: out <= 8'h20;
                    5'h1D: out <= 8'h20;
                    5'h1E: out <= 8'h20;
                    5'h1F: out <= 8'h20;

				default: out <= 8'h20;
				endcase
			end
			
			16'd13: // Resultado
			begin
				case (index)
					5'h00: out <= 8'h52;
					5'h01: out <= 8'h65;
					5'h02: out <= 8'h73;
					5'h03: out <= 8'h75;
					5'h04: out <= 8'h6c;
					5'h05: out <= 8'h74;
					5'h06: out <= 8'h61;
					5'h07: out <= 8'h64;
					5'h08: out <= 8'h6f;
				default: out <= 8'h20;
				endcase
			end
			
			16'd14: // Encerrar...
			begin
				case(index)
					5'h00: out <= 8'h46;
					5'h01: out <= 8'h69;
					5'h02: out <= 8'h6d;
					5'h03: out <= 8'h2e;
					5'h04: out <= 8'h2e;
					5'h05: out <= 8'h2e;
					default: out <= 8'h20;
				endcase
			end

			16'd15: // Executando S.O   ...
			begin
				case(index)
					5'h00: out <= 8'h45;
					5'h01: out <= 8'h78;
					5'h02: out <= 8'h65;
					5'h03: out <= 8'h63;
					5'h04: out <= 8'h75;
					5'h05: out <= 8'h74;
					5'h06: out <= 8'h61;
					5'h07: out <= 8'h6e;
					5'h08: out <= 8'h64;
					5'h09: out <= 8'h6f;
					5'h0A: out <= 8'h2e;
					5'h0B: out <= 8'h2e;
					5'h0C: out <= 8'h2e;
					5'h0D: out <= 8'h20;
					5'h0E: out <= 8'h20;
					5'h0F: out <= 8'h20;
					5'h10: out <= 8'h20;
					5'h11: out <= 8'h20;
					5'h12: out <= 8'h20;
					5'h13: out <= 8'h20;
					5'h14: out <= 8'h20;
					5'h15: out <= 8'h20;
					5'h16: out <= 8'h20;
					default: out <= 8'h20;
				endcase
			end
			16'd16: // Exeutando processo 
			begin
			    case (index)
			        5'h00: out <= 8'h45;
					5'h01: out <= 8'h78;
					5'h02: out <= 8'h65;
					5'h03: out <= 8'h63;
					5'h04: out <= 8'h75;
					5'h05: out <= 8'h74;
					5'h06: out <= 8'h61;
					5'h07: out <= 8'h6e;
					5'h08: out <= 8'h64;
					5'h09: out <= 8'h6f;
					5'h0A: out <= 8'h2e;
					5'h0B: out <= 8'h2e;
					5'h0C: out <= 8'h2e;
					5'h0D: out <= 8'h20;
					5'h0E: out <= 8'h20;
					5'h0F: out <= 8'h20;
					5'h10: out <= 8'h20;
					5'h11: out <= 8'h20;
					5'h12: out <= 8'h20;
					5'h13: out <= 8'h20;
					5'h14: out <= 8'h20;
					5'h15: out <= 8'h20;
					5'h16: out <= 8'h20;
	                default: out <= 8'h20;
				endcase
            end
			default:
				case (index)
					default: out <= 8'h20;
				endcase
			
		
		endcase 
		
	end //always
	  
endmodule 






/*
 GPIO_IN8 (GLOBAL RESET) resets LCD
ENTITY LCD_Display IS
-- Enter number of live Hex hardware data values to display
-- (do not count ASCII character constants)
    GENERIC(Num_Hex_Digits: Integer:= 2); 
-----------------------------------------------------------------------
-- LCD Displays 16 Characters on 2 lines
-- LCD_display string is an ASCII character string entered in hex for 
-- the two lines of the  LCD Display   (See ASCII to hex table below)
-- Edit LCD_Display_String entries above to modify display
-- Enter the ASCII character's 2 hex digit equivalent value
-- (see table below for ASCII hex values)
-- To display character assign ASCII value to LCD_display_string(x)
-- To skip a character use 8'h20" (ASCII space)
-- To dislay "live" hex values from hardware on LCD use the following: 
--   make array element for that character location 8'h0" & 4-bit field from Hex_Display_Data
--   state machine sees 8'h0" in high 4-bits & grabs the next lower 4-bits from Hex_Display_Data input
--   and performs 4-bit binary to ASCII conversion needed to print a hex digit
--   Num_Hex_Digits must be set to the count of hex data characters (ie. "00"s) in the display
--   Connect hardware bits to display to Hex_Display_Data input
-- To display less than 32 characters, terminate string with an entry of 8'hFE"
--  (fewer characters may slightly increase the LCD's data update rate)
------------------------------------------------------------------- 
--                        ASCII HEX TABLE
--  Hex                        Low Hex Digit
-- Value  0   1   2   3   4   5   6   7   8   9   A   B   C   D   E   F
------\----------------------------------------------------------------
--H  2 |  SP  !   "   #   $   %   &   '   (   )   *   +   ,   -   .   /
--i  3 |  0   1   2   3   4   5   6   7   8   9   :   ;   <   =   >   ?
--g  4 |  @   A   B   C   D   E   F   G   H   I   J   K   L   M   N   O
--h  5 |  P   Q   R   S   T   U   V   W   X   Y   Z   [   \   ]   ^   _
--   6 |  `   a   b   c   d   e   f   g   h   i   j   k   l   m   n   o
--   7 |  p   q   r   s   t   u   v   w   x   y   z   {   |   }   ~ DEL
-----------------------------------------------------------------------
-- Example "A" is row 4 column 1, so hex value is 8'h41"
-- *see LCD Controller's Datasheet for other graphics characters available
*/
        
module LCD_Display(iCLK_50MHZ, iRST_N, hex1, hex0, 
    LCD_RS,LCD_E,LCD_RW,DATA_BUS, reg_msg);
input iCLK_50MHZ, iRST_N;
input [3:0] hex1, hex0;
output LCD_RS, LCD_E, LCD_RW;
inout [7:0] DATA_BUS;
input [15:0] reg_msg;

parameter
HOLD = 4'h0,
FUNC_SET = 4'h1,
DISPLAY_ON = 4'h2,
MODE_SET = 4'h3,
Print_String = 4'h4,
LINE2 = 4'h5,
RETURN_HOME = 4'h6,
DROP_LCD_E = 4'h7,
RESET1 = 4'h8,
RESET2 = 4'h9,
RESET3 = 4'ha,
DISPLAY_OFF = 4'hb,
DISPLAY_CLEAR = 4'hc;

reg [3:0] state, next_command;
// Enter new ASCII hex data above for LCD Display
reg [7:0] DATA_BUS_VALUE;
wire [7:0] Next_Char;
reg [19:0] CLK_COUNT_400HZ;
reg [4:0] CHAR_COUNT;
reg CLK_400HZ, LCD_RW_INT, LCD_E, LCD_RS;

// BIDIRECTIONAL TRI STATE LCD DATA BUS
assign DATA_BUS = (LCD_RW_INT? 8'bZZZZZZZZ: DATA_BUS_VALUE);

LCD_display_string u1(
	.index(CHAR_COUNT),
	.out(Next_Char),
	.hex1(hex1),
	.hex0(hex0),
	.reg_msg(reg_msg)
);

assign LCD_RW = LCD_RW_INT;

always @(posedge iCLK_50MHZ or negedge iRST_N)
    if (!iRST_N)
    begin
       CLK_COUNT_400HZ <= 20'h00000;
       CLK_400HZ <= 1'b0;
    end
    else if (CLK_COUNT_400HZ < 20'h0F424)
    begin
       CLK_COUNT_400HZ <= CLK_COUNT_400HZ + 1'b1;
    end
    else
    begin
      CLK_COUNT_400HZ <= 20'h00000;
      CLK_400HZ <= ~CLK_400HZ;
    end
// State Machine to send commands and data to LCD DISPLAY

always @(posedge CLK_400HZ or negedge iRST_N)
    if (!iRST_N)
    begin
     state <= RESET1;
    end
    else
    case (state)
    RESET1:            
// Set Function to 8-bit transfer and 2 line display with 5x8 Font size
// see Hitachi HD44780 family data sheet for LCD command and timing details
    begin
      LCD_E <= 1'b1;
      LCD_RS <= 1'b0;
      LCD_RW_INT <= 1'b0;
      DATA_BUS_VALUE <= 8'h38;
      state <= DROP_LCD_E;
      next_command <= RESET2;
      CHAR_COUNT <= 5'b00000;
    end
    RESET2:
    begin
      LCD_E <= 1'b1;
      LCD_RS <= 1'b0;
      LCD_RW_INT <= 1'b0;
      DATA_BUS_VALUE <= 8'h38;
      state <= DROP_LCD_E;
      next_command <= RESET3;
    end
    RESET3:
    begin
      LCD_E <= 1'b1;
      LCD_RS <= 1'b0;
      LCD_RW_INT <= 1'b0;
      DATA_BUS_VALUE <= 8'h38;
      state <= DROP_LCD_E;
      next_command <= FUNC_SET;
    end
// EXTRA STATES ABOVE ARE NEEDED FOR RELIABLE PUSHBUTTON RESET OF LCD

    FUNC_SET:
    begin
      LCD_E <= 1'b1;
      LCD_RS <= 1'b0;
      LCD_RW_INT <= 1'b0;
      DATA_BUS_VALUE <= 8'h38;
      state <= DROP_LCD_E;
      next_command <= DISPLAY_OFF;
    end

// Turn off Display and Turn off cursor
    DISPLAY_OFF:
    begin
      LCD_E <= 1'b1;
      LCD_RS <= 1'b0;
      LCD_RW_INT <= 1'b0;
      DATA_BUS_VALUE <= 8'h08;
      state <= DROP_LCD_E;
      next_command <= DISPLAY_CLEAR;
    end

// Clear Display and Turn off cursor
    DISPLAY_CLEAR:
    begin
      LCD_E <= 1'b1;
      LCD_RS <= 1'b0;
      LCD_RW_INT <= 1'b0;
      DATA_BUS_VALUE <= 8'h01;
      state <= DROP_LCD_E;
      next_command <= DISPLAY_ON;
    end

// Turn on Display and Turn off cursor
    DISPLAY_ON:
    begin
      LCD_E <= 1'b1;
      LCD_RS <= 1'b0;
      LCD_RW_INT <= 1'b0;
      DATA_BUS_VALUE <= 8'h0C;
      state <= DROP_LCD_E;
      next_command <= MODE_SET;
    end

// Set write mode to auto increment address and move cursor to the right
    MODE_SET:
    begin
      LCD_E <= 1'b1;
      LCD_RS <= 1'b0;
      LCD_RW_INT <= 1'b0;
      DATA_BUS_VALUE <= 8'h06;
      state <= DROP_LCD_E;
      next_command <= Print_String;
    end

// Write ASCII hex character in first LCD character location
    Print_String:
    begin
      state <= DROP_LCD_E;
      LCD_E <= 1'b1;
      LCD_RS <= 1'b1;
      LCD_RW_INT <= 1'b0;
    // ASCII character to output
      if (Next_Char[7:4] != 4'h0)
        DATA_BUS_VALUE <= Next_Char;
        // Convert 4-bit value to an ASCII hex digit
      else if (Next_Char[3:0] >9)
        // ASCII A...F
         DATA_BUS_VALUE <= {4'h4,Next_Char[3:0]-4'h9};
      else
        // ASCII 0...9
         DATA_BUS_VALUE <= {4'h3,Next_Char[3:0]};
    // Loop to send out 32 characters to LCD Display  (16 by 2 lines)
      if ((CHAR_COUNT < 31) && (Next_Char != 8'hFE))
         CHAR_COUNT <= CHAR_COUNT + 1'b1;
      else
         CHAR_COUNT <= 5'b00000; 
    // Jump to second line?
      if (CHAR_COUNT == 15)
        next_command <= LINE2;
    // Return to first line?
      else if ((CHAR_COUNT == 31) || (Next_Char == 8'hFE))
        next_command <= RETURN_HOME;
      else
        next_command <= Print_String;
    end

// Set write address to line 2 character 1
    LINE2:
    begin
      LCD_E <= 1'b1;
      LCD_RS <= 1'b0;
      LCD_RW_INT <= 1'b0;
      DATA_BUS_VALUE <= 8'hC0;
      state <= DROP_LCD_E;
      next_command <= Print_String;
    end

// Return write address to first character postion on line 1
    RETURN_HOME:
    begin
      LCD_E <= 1'b1;
      LCD_RS <= 1'b0;
      LCD_RW_INT <= 1'b0;
      DATA_BUS_VALUE <= 8'h80;
      state <= DROP_LCD_E;
      next_command <= Print_String;
    end

// The next three states occur at the end of each command or data transfer to the LCD
// Drop LCD E line - falling edge loads inst/data to LCD controller
    DROP_LCD_E:
    begin
      LCD_E <= 1'b0;
      state <= HOLD;
    end
// Hold LCD inst/data valid after falling edge of E line                
    HOLD:
    begin
      state <= next_command;
    end
    endcase
endmodule


module LCD (
  input CLOCK_50, clk_div, lcd_trd_msg,   //    50 MHz clock
  input [7:0] GPIO_IN,    //    Toggle GPIO_INitch[17:0]
  input [15:0] offset,
  
  
//    LCD Module 16X2
  output LCD_ON,    // LCD Power ON/OFF
  output LCD_BLON,    // LCD Back Light ON/OFF
  output LCD_RW,    // LCD Read/Write Select, 0 = Write, 1 = Read
  output LCD_EN,    // LCD Enable
  output LCD_RS,    // LCD Command/Data Select, 0 = Command, 1 = Data
  inout [7:0] LCD_DATA    // LCD Data bus 8 bits
);

//    All inout port turn to tri-state

// reset delay gives some time for peripherals to initialize
wire DLY_RST;
Reset_Delay r0(.iCLK(CLOCK_50),.oRESET(DLY_RST) );

reg [15:0] reg_msg = 0;

always @ (posedge clk_div)
begin
	if(lcd_trd_msg)
		reg_msg <= offset;
end


// turn LCD ON
assign    LCD_ON        =    1'b1;
assign    LCD_BLON    =    1'b1;

wire [3:0] hex1, hex0;
assign hex1 = GPIO_IN[7:4];
assign hex0 = GPIO_IN[3:0];


LCD_Display u1(
// Host Side
   .iCLK_50MHZ(CLOCK_50),
   .iRST_N(DLY_RST),
   .hex0(hex0),
   .hex1(hex1),
	.reg_msg(reg_msg), ////
// LCD Side
   .DATA_BUS(LCD_DATA),
   .LCD_RW(LCD_RW),
   .LCD_E(LCD_EN),
   .LCD_RS(LCD_RS)
);

endmodule 