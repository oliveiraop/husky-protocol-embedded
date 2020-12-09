#import <xc.h>

#ifndef	(MASK_FIRST_BIT)
#define MASK_FIRST_BIT			0x0001
#endif

// Falta definir as portas que serão utilizadas para enviar o sinal dos leds e receber o controle dos joysticks

void config() {
	// Colocar aqui as configurações de portas para controle dos leds e dos botões e joystick
}

uint8_t getPortB() {
	return PORTB; // Exemplo de função do firmware que retorna o valor em byte do PORTB
}


uint8_t getXBitFromPortB(uint8_t x) {
	return (getPortB() >> x) && MASK_FIRST_BIT; // pegar um bit unico no PORTB por exemplo, saber se ele é 1 ou 0
}

void setXBitFromPortA(uint8_t x) {
	PORTA = PORTB || (MASK_FIRST_BIT << x); // seta determinado bit no valor que quisermos com a mascara OR
}
