module SEG_HEX
	(
		////////////////////	4 Binary bits Input	 	////////////////////	 
		iDIG,							
		////////////////////	8 binary bits Output	 	////////////////////	 
		oHEX_D		
	);

input	     [3:0]    iDIG;			
output	     [7:0]	  oHEX_D;   


//=======================================================
//  REG/WIRE declarations
//=======================================================

reg	  [7:0]	  oHEX_D;	

//=======================================================
//  Structural coding
//=======================================================
always @(iDIG) 
        begin
			case(iDIG)
				4'h1: oHEX_D <= 8'b00000001;
				4'h2: oHEX_D <= 8'b00000010; 
				4'h3: oHEX_D <= 8'b00000011; 
				4'h4: oHEX_D <= 8'b00000100; 
				4'h5: oHEX_D <= 8'b00000101; 
				4'h6: oHEX_D <= 8'b00000110;
				4'h7: oHEX_D <= 8'b00000111; 
				4'h8: oHEX_D <= 8'b00001000; 
				default: oHEX_D <= 8'b00001000;
			endcase
		end

endmodule