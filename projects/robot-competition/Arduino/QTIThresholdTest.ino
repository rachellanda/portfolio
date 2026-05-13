// Two QTI Analog Threshold Test
// Arduino Uno / ATmega328P
// Register notation only
//
// LEFT QTI signal  = A5
// RIGHT QTI signal = A4
//
// Left threshold:  1023
// Right threshold: 400

#include <avr/io.h>

#define LEFT_QTI_BLACK_THRESHOLD 1023
#define RIGHT_QTI_BLACK_THRESHOLD 400

// ---------------- UART ----------------

void uart_init() {
    UBRR0H = 0;
    UBRR0L = 103;        // 9600 baud for 16 MHz Arduino Uno
    UCSR0B = 0b00001000; // TX enable
    UCSR0C = 0b00000110; // 8-bit data
}

void uart_send_char(char c) {
    while (!(UCSR0A & 0b00100000));
    UDR0 = c;
}

void uart_send_string(const char* str) {
    while (*str != '\0') {
        uart_send_char(*str);
        str++;
    }
}

void uart_send_number(long num) {
    char buffer[12];
    int i = 0;

    if (num == 0) {
        uart_send_char('0');
        return;
    }

    if (num < 0) {
        uart_send_char('-');
        num = -num;
    }

    while (num > 0) {
        buffer[i] = (num % 10) + '0';
        num = num / 10;
        i++;
    }

    while (i > 0) {
        i--;
        uart_send_char(buffer[i]);
    }
}

// ---------------- Delay ----------------

void delay_ms(int ms) {
    for (int i = 0; i < ms; i++) {
        for (volatile int j = 0; j < 1600; j++) {
            __asm__("nop");
        }
    }
}

// ---------------- ADC ----------------

void adc_init() {
    // AVcc reference, ADC right-adjusted
    ADMUX = 0b01000000;

    // Enable ADC, prescaler = 128
    ADCSRA = 0b10000111;
}

int adc_read(unsigned char channel) {
    // Select ADC channel, keep AVcc reference
    ADMUX = 0b01000000 | (channel & 0b00001111);

    // Start conversion
    ADCSRA |= 0b01000000;

    // Wait for conversion to finish
    while (ADCSRA & 0b01000000);

    return ADC;
}

// ---------------- QTI Helpers ----------------

int read_left_qti() {
    // A5 = ADC5
    return adc_read(5);
}

int read_right_qti() {
    // A4 = ADC4
    return adc_read(4);
}

int left_qti_black() {
    if (read_left_qti() >= LEFT_QTI_BLACK_THRESHOLD) {
        return 1;
    } else {
        return 0;
    }
}

int right_qti_black() {
    if (read_right_qti() >= RIGHT_QTI_BLACK_THRESHOLD) {
        return 1;
    } else {
        return 0;
    }
}

// ---------------- Main ----------------

int main() {
    uart_init();
    adc_init();

    // A4 = PC4 input
    // A5 = PC5 input
    DDRC &= 0b11001111;

    uart_send_string("Two QTI threshold test start\r\n");
    uart_send_string("LEFT=A5 threshold=1023, RIGHT=A4 threshold=400\r\n\r\n");

    while (1) {
        int left_value = read_left_qti();
        int right_value = read_right_qti();

        uart_send_string("LEFT A5 = ");
        uart_send_number(left_value);

        if (left_value >= LEFT_QTI_BLACK_THRESHOLD) {
            uart_send_string(" BLACK");
        } else {
            uart_send_string(" NOT_BLACK");
        }

        uart_send_string("    RIGHT A4 = ");
        uart_send_number(right_value);

        if (right_value >= RIGHT_QTI_BLACK_THRESHOLD) {
            uart_send_string(" BLACK");
        } else {
            uart_send_string(" NOT_BLACK");
        }

        uart_send_string("\r\n");

        delay_ms(100);
    }

    return 0;
}
