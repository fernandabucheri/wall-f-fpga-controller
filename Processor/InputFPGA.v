module InputFPGA (
	input iCLK,              // Clock de 50MHz
	input iRST_n,            // Reset ativo baixo
	input iIRDA,             // Entrada do receptor IR
	output wire [16:0] Data  // Saída de dados binários
);

//----------------------------------------------------------------------
// Sinais internos para os módulos IR_RECEIVER e SEG_HEX
//----------------------------------------------------------------------
	wire        ir_data_ready;
	wire [31:0] ir_data;
	wire [7:0]  seg_hex_output;

//----------------------------------------------------------------------
// Instanciação do módulo IR_RECEIVE_modified
//----------------------------------------------------------------------
	IR_RECEIVER ir_receiver_inst (
		 .iCLK(iCLK),
		 .iRST_n(iRST_n),
		 .iIRDA(iIRDA),
		 .oDATA_READY(ir_data_ready),
		 .oDATA(ir_data)
	);

//----------------------------------------------------------------------
// Instanciação do módulo SEG_HEX
//----------------------------------------------------------------------
	SEG_HEX seg_hex_inst (
		 .iDIG(ir_data[19:16]), // Usando posições do Key Code
		 .oHEX_D(seg_hex_output)
	);
	
	// Atribuindo saídas do módulo
	assign Data = {9'b0, seg_hex_output};
	
endmodule

//module InputFPGA (SW0, SW1, SW2, SW3, SW4, SW5, SW6, SW7, SW8, SW9, SW10, SW11, SW12, SW13, SW14, SW15, SW16, Data);
//
//	input SW0, SW1, SW2, SW3, SW4, SW5, SW6, SW7, SW8, SW9, SW10, SW11, SW12, SW13, SW14, SW15, SW16;
//	output reg [16:0] Data;
//	
//	
//	always @(*)
//	    begin
//			Data[16] = SW16;
//        	Data[15] = SW15;
//        	Data[14] = SW14;
//        	Data[13] = SW13;
//        	Data[12] = SW12;
//        	Data[11] = SW11;
//        	Data[10] = SW10;
//        	Data[9] = SW9;
//        	Data[8] = SW8;
//        	Data[7] = SW7;
//        	Data[6] = SW6;
//        	Data[5] = SW5;
//        	Data[4] = SW4;
//        	Data[3] = SW3;
//        	Data[2] = SW2;
//        	Data[1] = SW1;
//        	Data[0] = SW0;
//    	end
//
//endmodule