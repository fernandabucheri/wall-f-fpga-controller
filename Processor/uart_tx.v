module uart_tx #(
    // Parâmetros de Configuração do Módulo
    // Estes parâmetros definem as características operacionais do transmissor UART,
    // permitindo sua adaptação a diferentes ambientes de hardware e requisitos de comunicação.
    parameter CLK_FREQ  = 50_000_000, // Frequência do clock do sistema em Hertz (Hz).
                                      // Este valor é crucial para o cálculo preciso dos tempos de bit.
                                      // Exemplo: 50 MHz indica 50 milhões de ciclos de clock por segundo.
    parameter BAUD_RATE = 9600        // Taxa de transmissão de dados em bits por segundo (bps).
                                      // Define a velocidade na qual os bits são transmitidos pela linha serial.
                                      // Exemplo: 9600 bps significa 9600 bits transmitidos por segundo.
)(
    input wire          i_clk,         // Sinal de clock principal do sistema. Todas as operações síncronas
                                       // dentro deste módulo são sincronizadas com as bordas de subida deste clock.
    input wire          i_rst_n,       // Sinal de reset assíncrono, ativo em nível baixo.
                                       // Quando '0', força o módulo a um estado inicial conhecido, independentemente do clock.
    input wire          i_tx_start,    // Pulso de um ciclo de clock que sinaliza o início de uma nova transmissão.
                                       // A transmissão do byte 'i_tx_data' é iniciada na borda de subida do clock
                                       // seguinte à detecção deste pulso.
    input wire  [7:0]   i_tx_data,     // Dado de 8 bits (um byte) a ser transmitido via UART.
                                       // Este valor é capturado quando 'i_tx_start' é detectado.

    output reg          o_tx_serial,   // Linha de saída serial UART (TX). Os bits são transmitidos sequencialmente
                                       // por esta linha, seguindo o protocolo UART.
    output reg          o_tx_busy      // Sinalizador de status que indica se o transmissor UART está atualmente
                                       // envolvido em uma transmissão. '1' indica ocupado, '0' indica livre.
);

//----------------------------------------------------------------------
// Cálculo de Ciclos de Clock por Bit
//----------------------------------------------------------------------
// Este parâmetro local calcula o número de ciclos de clock necessários para a duração de um único bit
// na taxa de transmissão especificada (BAUD_RATE) e frequência de clock (CLK_FREQ).
// Este valor é fundamental para temporizar com precisão cada elemento do protocolo UART (start bit,
// data bits e stop bit).
// Exemplo: Para CLK_FREQ = 50 MHz e BAUD_RATE = 9600 bps, CLKS_PER_BIT = 50,000,000 / 9600 ≈ 5208 ciclos.
localparam CLKS_PER_BIT = CLK_FREQ / BAUD_RATE;

//----------------------------------------------------------------------
// Definição dos Estados da Máquina de Estados Finita (FSM)
//----------------------------------------------------------------------
// A FSM controla a sequência de operações para a transmissão UART, garantindo que cada fase
// do protocolo seja executada na ordem correta e com o temporização adequada.
localparam [2:0] IDLE      = 3'b000; // Estado de repouso. O transmissor aguarda um comando para iniciar a transmissão.
localparam [2:0] START_BIT = 3'b001; // Estado de transmissão do bit de início (start bit), que é sempre '0'.
                                      // Este bit precede os dados e sinaliza o início de um novo frame ao receptor.
localparam [2:0] DATA_BITS = 3'b010; // Estado de transmissão dos 8 bits de dados. Os bits são enviados
                                      // sequencialmente, do menos significativo (LSB) para o mais significativo (MSB).
localparam [2:0] STOP_BIT  = 3'b011; // Estado de transmissão do bit de parada (stop bit), que é sempre '1'.
                                      // Este bit segue os dados e sinaliza o fim do frame ao receptor.

//----------------------------------------------------------------------
// Declaração de Registradores Internos
//----------------------------------------------------------------------
// Estes registradores armazenam o estado atual da FSM e variáveis de controle,
// sendo atualizados de forma síncrona com o clock. Os pares '_reg' e '_next'
// implementam a lógica de estado e próximo estado.
reg [2:0]   state_reg, state_next;        // 'state_reg' armazena o estado atual da FSM.
                                          // 'state_next' armazena o próximo estado da FSM, calculado pela lógica combinacional.
reg [15:0]  clk_count_reg, clk_count_next;// 'clk_count_reg' é um contador de ciclos de clock, usado para temporizar
                                          // a duração de cada bit.
                                          // 'clk_count_next' é o próximo valor do contador.
reg [3:0]   bit_index_reg, bit_index_next;// 'bit_index_reg' armazena o índice do bit de dado que está sendo transmitido
                                          // no momento (de 0 a 7).
                                          // 'bit_index_next' é o próximo índice do bit.
reg [7:0]   tx_data_reg, tx_data_next;    // 'tx_data_reg' armazena o byte de dado a ser transmitido.
                                          // 'tx_data_next' é o próximo valor do dado (geralmente o mesmo, a menos que
                                          // uma nova transmissão seja iniciada).

//----------------------------------------------------------------------
// Bloco Sequencial: Atualização de Registradores
//----------------------------------------------------------------------
// Este bloco descreve o comportamento síncrono do módulo. Os registradores são atualizados
// apenas na borda de subida do clock (posedge i_clk) ou quando o reset assíncrono é ativado
// (negedge i_rst_n).
always @(posedge i_clk or negedge i_rst_n)
begin
    if (!i_rst_n) begin
        // Lógica de Reset Assíncrono:
        // Quando o sinal de reset (i_rst_n) está em nível baixo, todos os registradores
        // são inicializados para seus valores padrão, retornando o módulo ao estado IDLE.
        state_reg       <= IDLE;        // FSM retorna ao estado de espera.
        clk_count_reg   <= 16'd0;       // Contador de ciclos é zerado.
        bit_index_reg   <= 4'd0;        // Índice do bit é zerado.
        tx_data_reg     <= 8'd0;        // Dado de transmissão é limpo.
        o_tx_busy       <= 1'b0;        // O transmissor é sinalizado como não ocupado.
    end else begin
        // Lógica de Operação Normal:
        // Na borda de subida do clock, os registradores são atualizados com os valores
        // calculados pela lógica combinacional ('_next' values).
        state_reg       <= state_next;
        clk_count_reg   <= clk_count_next;
        bit_index_reg   <= bit_index_next;
        tx_data_reg     <= tx_data_next;
        // O sinal 'o_tx_busy' é ativado ('1') sempre que o próximo estado da FSM não for IDLE,
        // indicando que uma transmissão está em andamento ou prestes a começar.
        o_tx_busy       <= state_next != IDLE;
    end
end

//----------------------------------------------------------------------
// Bloco Combinacional: Lógica da Máquina de Estados (FSM)
//----------------------------------------------------------------------
// Este bloco define a lógica combinacional que determina o próximo estado da FSM
// e os próximos valores dos registradores de controle, com base no estado atual
// e nas entradas do módulo. O 'always @(*)' garante que este bloco seja reavaliado
// sempre que qualquer uma de suas entradas (sinais à direita das atribuições) mudar.
always @(*)
begin
    // Atribuições Padrão:
    // Inicializa os valores '_next' com os valores '_reg' correspondentes.
    // Isso serve como um valor padrão e ajuda a evitar a inferência de latches,
    // garantindo que todos os sinais tenham um valor definido em todas as condições.
    state_next      = state_reg;
    clk_count_next  = clk_count_reg;
    bit_index_next  = bit_index_reg;
    tx_data_next    = tx_data_reg;

    // Lógica de Transição de Estados:
    // A FSM transita entre os estados com base no estado atual e nas condições de entrada.
    case (state_reg)
        IDLE:
        begin
            // No estado IDLE, o transmissor está aguardando um comando de início.
            // Os contadores são zerados em preparação para uma nova transmissão.
            clk_count_next = 16'd0;
            bit_index_next = 4'd0;
            if (i_tx_start) begin
                // Se o pulso 'i_tx_start' for detectado, a transmissão é iniciada.
                state_next      = START_BIT;    // Transita para o estado de transmissão do Start Bit.
                tx_data_next    = i_tx_data;    // O dado de entrada é armazenado no registrador interno.
                clk_count_next  = 16'd0;        // O contador de ciclos é zerado para o início do Start Bit.
            end
        end

        START_BIT:
        begin
            // No estado START_BIT, o módulo transmite o bit de início ('0').
            // Este bit deve ser mantido na linha por 'CLKS_PER_BIT' ciclos de clock.
            if (clk_count_reg == CLKS_PER_BIT - 1) begin
                // Se o contador de ciclos atingiu o limite para a duração do bit,
                // a transição para o próximo estado ocorre.
                state_next      = DATA_BITS;    // Transita para o estado de transmissão dos bits de dados.
                clk_count_next  = 16'd0;        // O contador é zerado para o início do primeiro bit de dado.
                bit_index_next  = 4'd0;         // O índice do bit é zerado para começar do LSB (bit 0).
            end else begin
                // Caso contrário, o contador de ciclos é incrementado.
                clk_count_next = clk_count_reg + 1;
            end
        end

        DATA_BITS:
        begin
            // No estado DATA_BITS, os 8 bits do dado são transmitidos sequencialmente.
            // Cada bit é mantido na linha por 'CLKS_PER_BIT' ciclos de clock.
            if (clk_count_reg == CLKS_PER_BIT - 1) begin
                // Se o contador de ciclos atingiu o limite para a duração do bit atual,
                // prepara-se para o próximo bit ou para o Stop Bit.
                clk_count_next = 16'd0; // O contador é zerado para o próximo bit.
                if (bit_index_reg == 4'd7) begin
                    // Se o último bit de dado (bit 7) foi transmitido,
                    // a transição para o Stop Bit ocorre.
                    state_next = STOP_BIT;
                end else begin
                    // Caso contrário, o índice do bit é incrementado para o próximo bit de dado.
                    bit_index_next = bit_index_reg + 1;
                end
            end else begin
                // Se o tempo para o bit atual ainda não expirou, o contador é incrementado.
                clk_count_next = clk_count_reg + 1;
            end
        end

        STOP_BIT:
        begin
            // No estado STOP_BIT, o módulo transmite o bit de parada ('1').
            // Este bit também é mantido na linha por 'CLKS_PER_BIT' ciclos de clock.
            if (clk_count_reg == CLKS_PER_BIT - 1) begin
                // Se o contador de ciclos atingiu o limite para a duração do Stop Bit,
                // a transmissão é concluída.
                state_next      = IDLE;     // Retorna ao estado IDLE, aguardando uma nova transmissão.
                clk_count_next  = 16'd0;    // O contador é zerado.
            end else begin
                // Caso contrário, o contador de ciclos é incrementado.
                clk_count_next = clk_count_reg + 1;
            end
        end

        default:
        begin
            // Tratamento de Estado Inválido:
            // Em caso de um estado inesperado (o que não deve ocorrer em uma FSM bem projetada),
            // o módulo é forçado a retornar ao estado IDLE para garantir a estabilidade.
            state_next = IDLE;
        end
    endcase
end

//----------------------------------------------------------------------
// Bloco Combinacional: Geração da Saída Serial (o_tx_serial)
//----------------------------------------------------------------------
// Este bloco define o valor da linha de saída serial 'o_tx_serial' com base no estado atual da FSM.
// O protocolo UART exige que a linha esteja em nível alto ('1') quando ociosa,
// em nível baixo ('0') para o start bit, transmita os bits de dado, e retorne a nível alto ('1')
// para o stop bit.
always @(*)
begin
    case(state_reg)
        IDLE:       o_tx_serial = 1'b1;                        // No estado IDLE, a linha TX permanece em nível alto (ociosa).
        START_BIT:  o_tx_serial = 1'b0;                        // No estado START_BIT, a linha TX é forçada a nível baixo ('0').
        DATA_BITS:  o_tx_serial = tx_data_reg[bit_index_reg]; // No estado DATA_BITS, a linha TX assume o valor do bit
                                                               // de dado atual, determinado por 'bit_index_reg'.
        STOP_BIT:   o_tx_serial = 1'b1;                        // No estado STOP_BIT, a linha TX retorna a nível alto ('1').
        default:    o_tx_serial = 1'b1;                        // Em caso de estado inesperado, a linha TX é mantida em nível alto
                                                               // para evitar interferências.
    endcase
end

endmodule