module uart_rx #(
    // Parâmetros de Configuração do Módulo
    // Estes parâmetros definem as características operacionais do receptor UART.
    parameter CLK_FREQ  = 50_000_000, // Frequência do clock do sistema em Hertz (Hz).
                                      // Essencial para o cálculo preciso dos tempos de amostragem dos bits.
    parameter BAUD_RATE = 9600        // Taxa de transmissão de dados em bits por segundo (bps).
                                      // Define a velocidade esperada dos bits recebidos.
)(
    input wire          i_clk,         // Sinal de clock principal do sistema.
    input wire          i_rst_n,       // Sinal de reset assíncrono, ativo em nível baixo.
    input wire          i_rx_serial,   // Linha de entrada serial UART (RX).

    output reg  [7:0]   o_rx_data,     // Dado de 8 bits recebido. Válido quando o_rx_ready é '1'.
    output reg          o_rx_ready,    // Pulso de um ciclo de clock indicando que um novo byte foi recebido e está disponível em o_rx_data.
    output reg          o_rx_busy      // Sinalizador de status que indica se o receptor UART está atualmente
                                       // no processo de receber um frame. '1' indica ocupado, '0' indica livre.
);

//----------------------------------------------------------------------
// Cálculo de Ciclos de Clock por Bit e Ponto de Amostragem
//----------------------------------------------------------------------
// CLKS_PER_BIT: Número de ciclos de clock para a duração de um bit.
// CLKS_PER_BIT_DIV2: Metade do número de ciclos de clock por bit. Usado para amostrar o bit no meio de sua duração.
localparam CLKS_PER_BIT      = CLK_FREQ / BAUD_RATE;
localparam CLKS_PER_BIT_DIV2 = CLKS_PER_BIT / 2;

//----------------------------------------------------------------------
// Definição dos Estados da Máquina de Estados Finita (FSM)
//----------------------------------------------------------------------
// A FSM controla a sequência de operações para a recepção UART.
localparam [2:0] IDLE        = 3'b000; // Estado de repouso. Aguarda a borda de descida do start bit.
localparam [2:0] START_BIT   = 3'b001; // Estado de detecção e validação do start bit.
localparam [2:0] DATA_BITS   = 3'b010; // Estado de recepção dos 8 bits de dados.
localparam [2:0] STOP_BIT    = 3'b011; // Estado de detecção e validação do stop bit.

//----------------------------------------------------------------------
// Declaração de Registradores Internos
//----------------------------------------------------------------------
reg [2:0]   state_reg, state_next;        // Estado atual e próximo da FSM.
reg [15:0]  clk_count_reg, clk_count_next;// Contador de ciclos de clock para temporização de bits.
reg [3:0]   bit_index_reg, bit_index_next;// Índice do bit de dado sendo recebido (0 a 7).
reg [7:0]   rx_data_reg, rx_data_next;    // Registrador temporário para armazenar os bits de dado recebidos.

//----------------------------------------------------------------------
// Bloco Sequencial: Atualização de Registradores
//----------------------------------------------------------------------
always @(posedge i_clk or negedge i_rst_n)
begin
    if (!i_rst_n) begin
        // Reset Assíncrono: Inicializa todos os registradores.
        state_reg       <= IDLE;
        clk_count_reg   <= 16'd0;
        bit_index_reg   <= 4'd0;
        rx_data_reg     <= 8'd0;
        o_rx_data       <= 8'd0;
        o_rx_ready      <= 1'b0;
        o_rx_busy       <= 1'b0;
    end else begin
        // Operação Normal: Atualiza registradores com valores '_next'.
        state_reg       <= state_next;
        clk_count_reg   <= clk_count_next;
        bit_index_reg   <= bit_index_next;
        rx_data_reg     <= rx_data_next;
        o_rx_data       <= rx_data_next; // o_rx_data é atualizado com o dado final.
        o_rx_ready      <= (state_next == IDLE && state_reg == STOP_BIT); // Pulso de 'ready' quando a FSM transita de STOP_BIT para IDLE.
        o_rx_busy       <= (state_next != IDLE); // Ocupado quando não está no estado IDLE.
    end
end

//----------------------------------------------------------------------
// Bloco Combinacional: Lógica da Máquina de Estados (FSM)
//----------------------------------------------------------------------
always @(*)
begin
    // Atribuições Padrão:
    state_next      = state_reg;
    clk_count_next  = clk_count_reg;
    bit_index_next  = bit_index_reg;
    rx_data_next    = rx_data_reg;

    case (state_reg)
        IDLE:
        begin
            // Aguarda a borda de descida na linha RX (início do start bit).
            clk_count_next = 16'd0;
            bit_index_next = 4'd0;
            rx_data_next   = 8'd0;
            if (i_rx_serial == 1'b0) begin
                // Detectou uma borda de descida, transita para o estado START_BIT.
                state_next      = START_BIT;
                // Inicia o contador para amostrar o meio do start bit.
                clk_count_next  = 16'd0;
            end
        end

        START_BIT:
        begin
            // Amostra o start bit no meio de sua duração.
            if (clk_count_reg == CLKS_PER_BIT_DIV2 - 1) begin
                if (i_rx_serial == 1'b0) begin
                    // Start bit válido, transita para o estado DATA_BITS.
                    state_next      = DATA_BITS;
                    // Zera o contador para o primeiro bit de dado.
                    clk_count_next  = 16'd0;
                    bit_index_next  = 4'd0;
                end else begin
                    // Start bit inválido (não permaneceu '0'), retorna a IDLE.
                    state_next = IDLE;
                end
            end else begin
                // Continua contando.
                clk_count_next = clk_count_reg + 1;
            end
        end

        DATA_BITS:
        begin
            // Amostra os 8 bits de dados, um por um.
            if (clk_count_reg == CLKS_PER_BIT - 1) begin
                // Amostra o bit atual no meio de sua duração.
                rx_data_next[bit_index_reg] = i_rx_serial;
                clk_count_next = 16'd0; // Zera o contador para o próximo bit.

                if (bit_index_reg == 4'd7) begin
                    // Todos os 8 bits de dados foram recebidos, transita para o STOP_BIT.
                    state_next = STOP_BIT;
                end else begin
                    // Incrementa o índice para o próximo bit de dado.
                    bit_index_next = bit_index_reg + 1;
                end
            end else begin
                // Continua contando.
                clk_count_next = clk_count_reg + 1;
            end
        end

        STOP_BIT:
        begin
            // Amostra o stop bit no meio de sua duração.
            if (clk_count_reg == CLKS_PER_BIT - 1) begin
                if (i_rx_serial == 1'b1) begin
                    // Stop bit válido, a recepção do frame foi bem-sucedida.
                    state_next = IDLE; // Retorna a IDLE, pronto para a próxima recepção.
                end else begin
                    // Stop bit inválido (não permaneceu '1'), indica erro de frame.
                    state_next = IDLE; // Retorna a IDLE, mas o dado pode ser inválido.
                end
                clk_count_next = 16'd0; // Zera o contador.
            end else begin
                // Continua contando.
                clk_count_next = clk_count_reg + 1;
            end
        end

        default:
        begin
            // Estado inválido, retorna a IDLE.
            state_next = IDLE;
        end
    endcase
end

endmodule