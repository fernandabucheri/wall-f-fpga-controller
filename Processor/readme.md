# Modificações para Integração com Controle IR

Esta pasta contém **somente os arquivos modificados** do sistema computacional presente no repositório [`MIPS-Processor-and-OS`](https://github.com/gabrielaverza/MIPS-Processor-and-OS), com o objetivo de adaptá-lo para uso com um controle infravermelho (IR) no contexto do projeto atual.

## Descrição dos Arquivos

* **IR\_RECEIVER.v**
  Código original de exemplo extraído do manual da placa FPGA Altera DE2-115. Utilizado **sem modificações**. Responsável por decodificar sinais IR recebidos do controle remoto.

* **SEG\_HEX.v**
  Também proveniente do manual da DE2-115, este módulo foi **adaptado**. Em vez de gerar sinais para o display de 7 segmentos, agora traduz os valores hexadecimais recebidos em seus equivalentes binários, facilitando a integração com a arquitetura do sistema.

* **InputFPGA.v**
  Originalmente recebia entradas das chaves da FPGA. Foi **modificado** para que a entrada agora seja fornecida pelo sinal do controle IR decodificado, tornando possível o uso do controle remoto como dispositivo de entrada.

* **clock\_divider.v**
  **Alterado** para aumentar a frequência de instruções executadas por segundo, com o objetivo de tornar o sistema mais responsivo aos comandos IR. A nova frequência é a mesma utilizada no outro sistema FPGA, garantindo **sincronização entre os dois sistemas**.

* **control\_unit.v**
  **Modificada** para que a instrução `input` gere o sinal de controle `PCwrite = 1` de forma incondicional, eliminando a dependência do botão `enter` para avançar o programa. Com isso, a execução das instruções ocorre continuamente após a entrada.

* **instruction\_memory.v**
  **Adaptado** para que o sistema execute **sempre o programa 1**, sem preempção.
