// Two Analog QTI Sensors + One-Wire Ultrasonic + Two TCS3200 Color Sensors + Robot Movement
// Arduino Uno / ATmega328P
// Register notation only
//
// Ultrasonic:
// SIG = D2
//
// QTI sensors:
// LEFT QTI  = A5
// RIGHT QTI = A4
//
// LEFT color sensor:
// S2  = D5
// S3  = D6
// OUT = A1
//
// RIGHT color sensor:
// S2  = D13
// S3  = D7
// OUT = A0
//
// Motor driver pins:
// LEFT MOTOR DIR  = D8
// LEFT MOTOR PWM  = D9   OC1A
// RIGHT MOTOR PWM = D10  OC1B
// RIGHT MOTOR DIR = D11

#include <avr/io.h>

// ---------------- Color Codes ----------------

#define COLOR_BLUE   0
#define COLOR_YELLOW 1
#define COLOR_BLACK  2

// ---------------- Motor Speeds ----------------
// Your setup is inverted:
// lower PWM number = faster
// 255 = stop

#define FORWARD_SPEED 24
#define DRIVE_SPEED   200
#define EASE_FAST     40
#define EASE_SLOW     90
#define TURN_SPEED    40

#define TURN_90_TIME_MS 400
#define TURN_180_TIME_MS 950

#define OBJECT_THRESHOLD_COUNT 2500

// ---------------- QTI Thresholds ----------------
// Analog ADC range: 0 to 1023
// LEFT QTI = A5
// RIGHT QTI = A4

#define LEFT_QTI_BLACK_THRESHOLD 1023
#define RIGHT_QTI_BLACK_THRESHOLD 400

// ---------------- Random State ----------------

unsigned long random_state = 12345;

// ---------------- UART ----------------

void uart_init() {
    UBRR0H = 0;
    UBRR0L = 103;        // 9600 baud for 16 MHz Arduino Uno
    UCSR0B = 0b00001000; // TXEN0 = 1
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

void delay_us(int us) {
    for (int i = 0; i < us; i++) {
        for (volatile int j = 0; j < 16; j++) {
            __asm__("nop");
        }
    }
}

// ---------------- ADC ----------------

void adc_init() {
    // AVcc reference, ADC right adjusted
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

// ---------------- Pin + Timer Setup ----------------

void pins_init() {
    // PORTD:
    // D2 = PD2 = ultrasonic SIG, input/output dynamically
    // D5 = PD5 = LEFT color S2 output
    // D6 = PD6 = LEFT color S3 output
    // D7 = PD7 = RIGHT color S3 output
    DDRD = 0b11100000;

    // Pullup OFF for D2
    PORTD &= 0b11111011;

    // PORTB:
    // D8  = PB0 = LEFT MOTOR DIR output
    // D9  = PB1 = LEFT MOTOR PWM output OC1A
    // D10 = PB2 = RIGHT MOTOR PWM output OC1B
    // D11 = PB3 = RIGHT MOTOR DIR output
    // D13 = PB5 = RIGHT color S2 output
    DDRB = 0b00101111;

    // PORTC:
    // A0 = PC0 = RIGHT color OUT input
    // A1 = PC1 = LEFT color OUT input
    // A4 = PC4 = RIGHT QTI analog input
    // A5 = PC5 = LEFT QTI analog input
    DDRC = 0b00000000;
}

void timer1_pwm_init() {
    // Timer1 controls PWM on D9 = OC1A and D10 = OC1B
    TCCR1A = 0b10100001; // Fast PWM 8-bit, non-inverting
    TCCR1B = 0b00001011; // Prescaler = 64

    OCR1A = 255;
    OCR1B = 255;
}

// ---------------- Motor Control ----------------

void set_left_motor_forward() {
    PORTB |= 0b00000001; // D8 HIGH
}

void set_left_motor_backward() {
    PORTB &= 0b11111110; // D8 LOW
}

void set_right_motor_forward() {
    PORTB |= 0b00001000; // D11 HIGH
}

void set_right_motor_backward() {
    PORTB &= 0b11110111; // D11 LOW
}

void set_motor_speeds(unsigned char left_speed, unsigned char right_speed) {
    OCR1A = left_speed;   // D9
    OCR1B = right_speed;  // D10
}

void move_forward() {
    set_left_motor_forward();
    set_right_motor_forward();

    // Right motor slowed slightly so robot drives straight.
    set_motor_speeds(FORWARD_SPEED, FORWARD_SPEED + 24);
}

void move_backward() {
    set_left_motor_backward();
    set_right_motor_backward();

    // Tuned reverse correction
    set_motor_speeds(DRIVE_SPEED + 24, DRIVE_SPEED);
}

void stop_motors() {
    set_left_motor_forward();
    set_right_motor_forward();
    set_motor_speeds(255, 255);
}

void turn_right_90() {
    stop_motors();
    delay_ms(100);

    set_left_motor_forward();
    set_right_motor_backward();
    set_motor_speeds(TURN_SPEED, TURN_SPEED);

    delay_ms(TURN_90_TIME_MS);

    stop_motors();
    delay_ms(100);
}

void turn_left_90() {
    stop_motors();
    delay_ms(100);

    set_left_motor_backward();
    set_right_motor_forward();
    set_motor_speeds(TURN_SPEED, TURN_SPEED);

    delay_ms(TURN_90_TIME_MS);

    stop_motors();
    delay_ms(100);
}

void turn_180() {
    stop_motors();


    set_left_motor_forward();
    set_right_motor_backward();
    set_motor_speeds(TURN_SPEED, TURN_SPEED);

    delay_ms(TURN_180_TIME_MS);

    stop_motors();

}

// ---------------- Analog QTI Reading ----------------

int read_left_qti_raw() {
    // LEFT QTI = A5 = ADC5
    return adc_read(5);
}

int read_right_qti_raw() {
    // RIGHT QTI = A4 = ADC4
    return adc_read(4);
}

int left_qti_black() {
    int value = read_left_qti_raw();

    if (value >= LEFT_QTI_BLACK_THRESHOLD) {
        return 1;
    } else {
        return 0;
    }
}

int right_qti_black() {
    int value = read_right_qti_raw();

    if (value >= RIGHT_QTI_BLACK_THRESHOLD) {
        return 1;
    } else {
        return 0;
    }
}

void print_qti_values() {
    int left_value = read_left_qti_raw();
    int right_value = read_right_qti_raw();

    uart_send_string("LEFT QTI A5=");
    uart_send_number(left_value);

    if (left_value >= LEFT_QTI_BLACK_THRESHOLD) {
        uart_send_string(" BLACK");
    } else {
        uart_send_string(" NOT_BLACK");
    }

    uart_send_string("    RIGHT QTI A4=");
    uart_send_number(right_value);

    if (right_value >= RIGHT_QTI_BLACK_THRESHOLD) {
        uart_send_string(" BLACK");
    } else {
        uart_send_string(" NOT_BLACK");
    }

    uart_send_string("\r\n");
}

// ---------------- One-Wire Ultrasonic Helpers ----------------

void ultrasonic_pin_output() {
    DDRD |= 0b00000100; // D2 output
}

void ultrasonic_pin_input() {
    DDRD &= 0b11111011;  // D2 input
    PORTD &= 0b11111011; // pullup off
}

void ultrasonic_pin_high() {
    PORTD |= 0b00000100;
}

void ultrasonic_pin_low() {
    PORTD &= 0b11111011;
}

int read_ultrasonic_pin() {
    if (PIND & 0b00000100) {
        return 1;
    } else {
        return 0;
    }
}

long read_ultrasonic_raw() {
    long count = 0;
    long timeout = 0;

    ultrasonic_pin_output();

    ultrasonic_pin_low();
    delay_us(2);

    ultrasonic_pin_high();
    delay_us(10);

    ultrasonic_pin_low();

    ultrasonic_pin_input();

    timeout = 0;
    while (read_ultrasonic_pin() == 0) {
        timeout++;

        if (timeout > 30000) {
            return -1;
        }
    }

    count = 0;
    while (read_ultrasonic_pin() == 1) {
        count++;

        if (count > 30000) {
            return -2;
        }
    }

    return count;
}

int check_object() {
    long raw = read_ultrasonic_raw();

    if (raw == -1 || raw == -2) {
        return 0;
    }

    if (raw < OBJECT_THRESHOLD_COUNT) {
        return 1;
    } else {
        return 0;
    }
}

// ---------------- LEFT Color Sensor Filter Selection ----------------
// Color code is included but not used in main right now.

void left_select_red() {
    // LEFT S2 = D5 LOW
    // LEFT S3 = D6 LOW
    PORTD &= 0b10011111;
}

void left_select_green() {
    // LEFT S2 = D5 HIGH
    // LEFT S3 = D6 HIGH
    PORTD |= 0b01100000;
}

void left_select_blue() {
    // LEFT S2 = D5 LOW
    // LEFT S3 = D6 HIGH
    PORTD &= 0b11011111;
    PORTD |= 0b01000000;
}

// ---------------- RIGHT Color Sensor Filter Selection ----------------
// Color code is included but not used in main right now.

void right_select_red() {
    // RIGHT S2 = D13/PB5 LOW
    // RIGHT S3 = D7 LOW
    PORTB &= 0b11011111;
    PORTD &= 0b01111111;
}

void right_select_green() {
    // RIGHT S2 = D13/PB5 HIGH
    // RIGHT S3 = D7 HIGH
    PORTB |= 0b00100000;
    PORTD |= 0b10000000;
}

void right_select_blue() {
    // RIGHT S2 = D13/PB5 LOW
    // RIGHT S3 = D7 HIGH
    PORTB &= 0b11011111;
    PORTD |= 0b10000000;
}

// ---------------- Color Sensor Output Reading ----------------

int read_left_color_out() {
    // LEFT OUT = A1 = PC1
    if (PINC & 0b00000010) {
        return 1;
    } else {
        return 0;
    }
}

int read_right_color_out() {
    // RIGHT OUT = A0 = PC0
    if (PINC & 0b00000001) {
        return 1;
    } else {
        return 0;
    }
}

long read_left_color_pulse() {
    long count = 0;
    long timeout = 0;

    while (read_left_color_out() == 1) {
        timeout++;
        if (timeout > 30000) {
            return 30000;
        }
    }

    while (read_left_color_out() == 0) {
        count++;
        if (count > 30000) {
            return 30000;
        }
    }

    return count;
}

long read_right_color_pulse() {
    long count = 0;
    long timeout = 0;

    while (read_right_color_out() == 1) {
        timeout++;
        if (timeout > 30000) {
            return 30000;
        }
    }

    while (read_right_color_out() == 0) {
        count++;
        if (count > 30000) {
            return 30000;
        }
    }

    return count;
}

long read_left_color_avg(int samples) {
    long total = 0;

    for (int i = 0; i < samples; i++) {
        total += read_left_color_pulse();
    }

    return total / samples;
}

long read_right_color_avg(int samples) {
    long total = 0;

    for (int i = 0; i < samples; i++) {
        total += read_right_color_pulse();
    }

    return total / samples;
}

void read_left_rgb(long* r, long* g, long* b) {
    left_select_red();
    delay_ms(10);
    *r = read_left_color_avg(1);

    left_select_green();
    delay_ms(10);
    *g = read_left_color_avg(1);

    left_select_blue();
    delay_ms(10);
    *b = read_left_color_avg(1);
}

void read_right_rgb(long* r, long* g, long* b) {
    right_select_red();
    delay_ms(10);
    *r = read_right_color_avg(1);

    right_select_green();
    delay_ms(10);
    *g = read_right_color_avg(1);

    right_select_blue();
    delay_ms(10);
    *b = read_right_color_avg(1);
}

int detect_color_code(long r, long g, long b) {
    long diff_blue = (r - 330) * (r - 330)
                   + (g - 180) * (g - 180)
                   + (b - 80)  * (b - 80);

    long diff_yellow = (r - 30) * (r - 30)
                     + (g - 50) * (g - 50)
                     + (b - 100) * (b - 100);

    long diff_black = (r - 550) * (r - 550)
                    + (g - 600) * (g - 600)
                    + (b - 480) * (b - 480);

    if (diff_black <= diff_blue && diff_black <= diff_yellow) {
        return COLOR_BLACK;
    }

    if (diff_blue <= diff_yellow && diff_blue <= diff_black) {
        return COLOR_BLUE;
    }

    return COLOR_YELLOW;
}

const char* color_name(int color_code) {
    if (color_code == COLOR_BLACK) {
        return "BLACK";
    }

    if (color_code == COLOR_BLUE) {
        return "BLUE";
    }

    return "YELLOW";
}

void print_color_values_once() {
    long left_r;
    long left_g;
    long left_b;

    long right_r;
    long right_g;
    long right_b;

    read_left_rgb(&left_r, &left_g, &left_b);
    read_right_rgb(&right_r, &right_g, &right_b);

    int left_color = detect_color_code(left_r, left_g, left_b);
    int right_color = detect_color_code(right_r, right_g, right_b);

    uart_send_string("LEFT COLOR: ");
    uart_send_string(color_name(left_color));
    uart_send_string(" R=");
    uart_send_number(left_r);
    uart_send_string(" G=");
    uart_send_number(left_g);
    uart_send_string(" B=");
    uart_send_number(left_b);

    uart_send_string("     RIGHT COLOR: ");
    uart_send_string(color_name(right_color));
    uart_send_string(" R=");
    uart_send_number(right_r);
    uart_send_string(" G=");
    uart_send_number(right_g);
    uart_send_string(" B=");
    uart_send_number(right_b);

    uart_send_string("\r\n");
}

// ---------------- Random Direction ----------------

unsigned long simple_random() {
    random_state = random_state * 1103515245 + 12345;
    return random_state;
}

void random_small_turn() {
    unsigned long r = simple_random() % 3;

    if (r == 0) {
        turn_left_90();
    }
    else if (r == 1) {
        turn_right_90();
    }
    else {
        move_forward();
        delay_ms(150);
    }
}

// ---------------- Fast QTI Competition Logic ----------------

void handle_qti_event() {
    int left_black = left_qti_black();
    int right_black = right_qti_black();

    if (left_black && right_black) {
        uart_send_string("BOTH QTI BLACK: 180\r\n");
        turn_180();
        random_small_turn();
    }
    else if (left_black) {
        uart_send_string("LEFT QTI BLACK: RIGHT 90\r\n");
        turn_right_90();
        random_small_turn();
    }
    else if (right_black) {
        uart_send_string("RIGHT QTI BLACK: LEFT 90\r\n");
        turn_left_90();
        random_small_turn();
    }
}

// ---------------- Main ----------------

int main() {
    pins_init();
    timer1_pwm_init();
    uart_init();
    adc_init();

    stop_motors();
    delay_ms(100);

    uart_send_string("START\r\n");

    // Start going straight
    move_forward();
    delay_ms(300);

    // Choose random turn ONLY ONCE at the beginning
    uart_send_string("INITIAL RANDOM TURN\r\n");
    random_small_turn();

    // Then only drive straight until black, turn 180, repeat
    while (1) {
        int left_black = left_qti_black();
        int right_black = right_qti_black();

        if (left_black || right_black) {
            uart_send_string("BLACK DETECTED: TURN 180\r\n");

            stop_motors();
            delay_ms(100);

            move_backward();
            delay_ms(150);


            turn_180();

            // Drive forward after turning away from black
            move_forward();
            delay_ms(300);
        }
        else {
            move_forward();
        }

        // Fast QTI sampling
        delay_ms(5);
    }

    return 0;
}
