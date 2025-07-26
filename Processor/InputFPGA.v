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