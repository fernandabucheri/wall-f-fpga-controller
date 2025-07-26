module instruction_memory
#(parameter DATA_WIDTH=32,
parameter ADDR_WIDTH=12 /* para endereçar até 4096 posições */)
(
	input [(ADDR_WIDTH-1):0] addr,
	input clk, 
	output reg [(DATA_WIDTH-1):0] q
);

	reg [DATA_WIDTH-1:0] rom[2**ADDR_WIDTH-1:0]; 

	initial
	begin
      // Carrega cada processo em seu intervalo especifico
	  $readmemb("jinit.txt", rom, 0);                // Salto para inicializacao
 //   $readmemb("proc1.txt", rom, 1, 100);           // Processo 1 - Contador crescente
	  $readmemb("proc_v2.txt", rom, 1, 100);         // Processo 1 - Auxiliar - Sistema CD
      $readmemb("proc2.txt", rom, 101, 200);         // Processo 2 - Contador decrescente
      $readmemb("proc3.txt", rom, 201, 300);         // Processo 3 - Eh par
      $readmemb("proc4.txt", rom, 301, 400);         // Processo 4 - Eh primo
      $readmemb("proc5.txt", rom, 401, 500);         // Processo 5 - Fatorial
      $readmemb("proc6.txt", rom, 501, 600);         // Processo 6 - Fibonacci
      $readmemb("proc7.txt", rom, 601, 700);         // Processo 7 - Média
      $readmemb("proc8.txt", rom, 701, 800);         // Processo 8 - Potência
      $readmemb("proc9.txt", rom, 801, 900);         // Processo 9 - Resto
      $readmemb("proc10.txt", rom, 901, 1000);       // Processo 10 - Área triângulo
        
      // Carrega as particoes de controle do SO
//    $readmemb("main_so.txt", rom, 1024, 1035);             // Main SO
      $readmemb("main_so_v2.txt", rom, 1024, 1035);          // Main SO - sempre sem preempcao
      $readmemb("context_switch.txt", rom, 1124, 1189);      // Troca de contexto
      $readmemb("proc_handlers.txt", rom, 1192, 1231);       // Handlers dos processos
      $readmemb("proc_manager.txt", rom, 1250);              // Chama gerenciador de processos
//    $readmemb("non_preemptive.txt", rom, 1830, 1873);      // Execução sem preempção
	  $readmemb("non_preemptive_v2.txt", rom, 1830, 1873);   // Execução sem preempção - executa sempre programa 1
	  $readmemb("preemptive.txt", rom, 1900, 1914);          // Execução com preempção
	  $readmemb("init.txt", rom, 1920, 1930);                // Inicializa ram e vai para main SO
	end

	always @ (posedge clk)
	begin
		q <= rom[addr];
	end

endmodule