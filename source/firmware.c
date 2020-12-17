#import <xc.h>

#ifndef	(MASK_FIRST_BIT)
#define MASK_FIRST_BIT			0x0001
#endif

// Falta definir as portas que serão utilizadas para enviar o sinal dos leds e receber o controle dos joysticks

void config() {
	// Colocar aqui as configurações de portas para controle dos leds e dos botões e joystick
	TRISFSET = 0x013F; //Set RF 1-5 e RF8 input
	TRISACLR = 0x00FF; // Clear RA0 até RA7 para output
	AD1PCFGSET = 0xFFFF; // transforma pinos analogicos em digitais
	
}

uint8_t getPortA() {
	return PORTA; // Exemplo de função do firmware que retorna o valor em byte do PORTB
}


uint8_t getXBitFromPortF(uint8_t x) {
	return (getPortF() >> x) && MASK_FIRST_BIT; // pegar um bit unico no PORTB por exemplo, saber se ele é 1 ou 0
}

void setXBitFromPortA(uint8_t x) {
	PORTASET = (MASK_FIRST_BIT << x); // seta determinado bit no valor que quisermos com a mascara OR
}
