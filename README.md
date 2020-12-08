# husky-protocol-embedded

Projeto de um microcontrolador para o robô Husky UGV da Clearpath, utilizando FreeRTOS em um PIC32MX.

## Estrutura

Consiste de arquivos incluídos e modificados do Demo PIC32MX_MPLAB, que acompanha a instalação do FreeRTOS.

* `main.c` e `main_full.c`: arquivos de execução do sistema, modificados do exemplo original
* `Message.c`: funções de definição e manipulação das Mensagens a serem trocadas pelo sistema
* `crc.c`: função de Checksum CRC CCITT por tabela

## Instalação

Os arquivos da pasta `source/` devem ser copiados para a pasta `Demon/PIC32MX_MPLAB/` do FreeRTOS, substituindo quaisquer arquivos em conflito. O projeto deve ser executado e compilado pelo MPLAB X e MPLAB XC32.

## Referências

* [FreeRTOS Microchip PIC32 MX RTOS port](https://www.freertos.org/port_PIC32_MIPS_MK4.html)
* [Clearpath Communication Protocol](https://www.clearpathrobotics.com/assets/downloads/communication_protocols.pdf)
* [PIC32M Family Reference Manual](https://microchipdeveloper.com/32bit:frm)