#include "stm32f4xx.h"  // Device header

// Ultrasonic sensor variables
void GPIO_Config(void);
void delay_us(uint32_t us);
void interrupt_config(void);
void echo_generate(void);
void TIM2_Config(void);
void USART2_Config(void);
int distance_cm;
volatile uint32_t start_time = 0;
volatile uint32_t end_time = 0;

// FSR variables
uint16_t adc_value;  // Variable to store ADC value for FSR
int food_message_sent = 0;     // Flag to track if the "Food quantity greater than 2000" message was sent
int no_food_message_sent = 0;  // Flag to track if the "No food in tray" message was sent
int optimum_food_message_sent = 0; 
void ADC_Init(void);
uint16_t ADC_Read(void);
void delay(volatile uint32_t delay);  // Delay function for FSR

int prev_adc_nonzero = 1;  // Track previous ADC state
const int FOOD_THRESHOLD_HIGH = 2000;
const int FOOD_THRESHOLD_LOW = 1;

// Door state variables
int door_state = 0;  // 0 = closed, 1 = open
int door_message_sent = 0;  // Flag to track if the door message was sent

int main(void) {
    GPIO_Config();          // Initialize GPIO pins for TRIG, ECHO, and LEDs
    interrupt_config();      // Configuring an interrupt for the ECHO pin
    TIM2_Config();           // Configuring timer for ultrasonic sensor
    ADC_Init();              // Initialize ADC for FSR
    USART2_Config();         // Initialize USART2 for Bluetooth communication

    while (1) {
        // Ultrasonic sensor operations
        echo_generate();  // Generate echo for ultrasonic sensor

        // Check distance and update door state
        if (distance_cm < 150) {
            GPIOA->ODR |= (1U << 10);  // Turn on LED connected to PA10
						delay(1000);
            if (door_state == 0) {  // If the door is currently closed
                door_state = 1;  // Update state to open
                door_message_sent = 0;  // Reset door message sent flag
            }
            if (!door_message_sent) {
                // Send "door open" message
                char message[] = "door open\r\n";
                for (int i = 0; message[i] != '\0'; i++) {
                    while (!(USART2->SR & (1U << 7))) {};  // Wait until TXE (Transmit Data Register Empty)
                    USART2->DR = message[i];               // Send the current character
                }
                door_message_sent = 1;  // Set the message flag
            }
        } else {
            GPIOA->ODR &= ~(1U << 10); // Turn off LED connected to PA10
            if (door_state == 1) {  // If the door is currently open
                door_state = 0;  // Update state to closed
                door_message_sent = 0;  // Reset door message sent flag
            }
            if (!door_message_sent) {
                // Send "door closed" message
                char message[] = "door closed\r\n";
                for (int i = 0; message[i] != '\0'; i++) {
                    while (!(USART2->SR & (1U << 7))) {};  // Wait until TXE (Transmit Data Register Empty)
                    USART2->DR = message[i];               // Send the current character
                }
                door_message_sent = 1;  // Set the message flag
            }
        }

        // FSR operations
        adc_value = ADC_Read();  // Read ADC value from FSR

    if (adc_value > 2000 && food_message_sent == 0) {
			GPIOA->ODR &= ~(1U << 8);
        char message[] = "Food quantity greater than 2000\r\n";
        for (int i = 0; message[i] != '\0'; i++) {
            while (!(USART2->SR & (1U << 7))) {};  
            USART2->DR = message[i];               
        }
        food_message_sent = 1;         
        no_food_message_sent = 0;      
        optimum_food_message_sent = 0; // Reset optimum message flag
        prev_adc_nonzero = 1;          
    } 
    else if (adc_value <500 && prev_adc_nonzero == 1) {
			GPIOA->ODR |= (1U << 8);
        char message[] = "No food in tray\r\n";
        for (int i = 0; message[i] != '\0'; i++) {
            while (!(USART2->SR & (1U << 7))) {};  
            USART2->DR = message[i];               
        }
				
        no_food_message_sent = 1;       
        food_message_sent = 0;          
        optimum_food_message_sent = 0; // Reset optimum message flag
        prev_adc_nonzero = 0;           // Update previous value tracker to zero
    }
    else if (adc_value >=500 && adc_value <= 2000) {
        // Check for optimum food quantity
			GPIOA->ODR &= ~(1U << 8);
        if (!optimum_food_message_sent) {
            char message[] = "The food is optimum\r\n";
            for (int i = 0; message[i] != '\0'; i++) {
                while (!(USART2->SR & (1U << 7))) {};  
                USART2->DR = message[i];               
            }
            optimum_food_message_sent = 1; // Set the optimum message flag
        }
        food_message_sent = 0;
        no_food_message_sent = 0;
        prev_adc_nonzero = 1;  
    }

        delay(10000);  
    }
}


// Function to configure GPIO pins
void GPIO_Config(void) {
    // Enable GPIOA and GPIOB clocks
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN;

    // Set PB0 (TRIG) as output
    GPIOB->MODER |= (1 << 0);       // Set PB0 as output
    GPIOB->OTYPER &= ~(1 << 0);     // Output push-pull
    GPIOB->OSPEEDR |= (3 << 0);     // Set high speed

    // Set PB1 (ECHO) as input
    GPIOB->MODER &= ~(3 << 2);      // Set PB1 as input

    // Set PA4, PA5, PA6, PA10 as output for LEDs
    GPIOA->MODER |= (1 << 8) | (1 << 10) | (1 << 12) | (1 << 20)| (1 << 16);
	

    // Set PA1 as analog mode for FSR
    GPIOA->MODER |= (3UL << (1 * 2));  // Set PA1 as analog mode (for ADC input)
}

// Configuring the interrupt pin PB1
void interrupt_config() {
    RCC->APB2ENR |= (1U << 14); // Enable system configuration controller clock

    // Map the interrupt to PB1
    SYSCFG->EXTICR[0] = 0;             // Clear bits for safety
    SYSCFG->EXTICR[0] |= (1U << 4);    // Map EXTI1 to PB1

    EXTI->IMR |= (1U << 1);            // Unmask the interrupt at PB1
    EXTI->FTSR |= (1U << 1);           // Select falling edge trigger for PB1
    EXTI->RTSR |= (1U << 1);           // Select rising edge trigger for PB1
    NVIC_EnableIRQ(EXTI1_IRQn);        // Enable the NVIC for EXTI1
}

// Function to create a delay in microseconds using SysTick
void delay_us(uint32_t us) {
    volatile uint32_t count = 84 * us;  // 1 microsecond delay at 84 MHz
    while (count--) {
        __NOP();  // No operation (dummy instruction)
    }
}

// Function to measure distance using the ultrasonic sensor
void echo_generate(void) {
    // Send a 10us pulse to TRIG pin to start ultrasonic burst
    GPIOB->ODR |= (1 << 0);  // Set PB0 HIGH
    delay_us(10);            // Wait for 10 microseconds
    GPIOB->ODR &= ~(1 << 0);  // Set PB0 LOW
}

void EXTI1_IRQHandler(void) {
    if (EXTI->PR & (1U << 1)) {  // Check if interrupt was caused by PB1
        EXTI->PR |= (1U << 1);  // Clear the pending interrupt flag

        if (GPIOB->IDR & (1 << 1)) {  // Rising edge detected (echo pulse started)
            TIM2->CNT = 0;  // Reset TIM2 counter
            TIM2->CR1 |= TIM_CR1_CEN;  // Start TIM2
        } else {  // Falling edge detected (echo pulse ended)
            TIM2->CR1 &= ~TIM_CR1_CEN;  // Stop TIM2
            end_time = TIM2->CNT;  // Capture end time (pulse duration in microseconds)

            // Convert pulse duration to distance in cm
            distance_cm = (end_time * 0.0343) / 2;  // Speed of sound = 0.0343 cm/us, divide by 2 for round trip
        }
    }
}

// Setting the timer for ultrasonic sensor
void TIM2_Config(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;  // Enable TIM2 clock

    TIM2->PSC = 84 - 1;  // Prescaler to slow down the clock (84 MHz / 84 = 1 MHz)
    TIM2->ARR = 0xFFFFFFFF;  // Set auto-reload to maximum (32-bit timer)
    TIM2->CNT = 0;  // Reset counter
    TIM2->CR1 |= TIM_CR1_URS;  // Update request source
}

// ADC initialization for FSR
void ADC_Init(void) {
    // Enable GPIOA and ADC1 clocks
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;  // Enable GPIOA clock
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;   // Enable ADC1 clock

    // ADC configuration
    ADC1->CR2 |= (1U << 1);                       // Start with ADC disabled (clear ADON bit)
    ADC1->SQR3 = 1;                      // Select channel 1 for ADC conversion
    ADC1->SMPR2 |= (7 << 3);             // Set sample time for channel 1 to 480 cycles (maximum)
    ADC1->CR2 |= ADC_CR2_ADON;           // Enable the ADC
}

// Function to read ADC value for FSR
uint16_t ADC_Read(void) {
    ADC1->CR2 |= ADC_CR2_SWSTART;        // Start the ADC conversion
    while (!(ADC1->SR & ADC_SR_EOC));    // Wait until conversion is complete
    return ADC1->DR;                     // Read the ADC conversion result
}

// USART2 configuration for Bluetooth module
void USART2_Config(void) {
    RCC->AHB1ENR |= (1 << 0);    // Enable GPIOA clock
    GPIOA->MODER |= (1 << 5);    // Set PA2 to alternate function mode for USART2 Tx
    GPIOA->AFR[0] |= (7 << 8);   // Set AF7 for USART2 on PA2

    RCC->APB1ENR |= (1 << 17);   // Enable USART2 clock
    USART2->BRR = 0x683;         // Set baud rate to 115200 @ 16 MHz
    USART2->CR1 |= (1 << 3);     // Enable transmitter
    USART2->CR1 |= (1 << 13);    // Enable USART2
}

// Simple delay for debounce
void delay(volatile uint32_t delay) {
    while(delay--);
}
