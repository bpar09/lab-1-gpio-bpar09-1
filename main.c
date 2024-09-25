/**
  ******************************************************************************
  * @file    main.c
  * @author  Biak Par
  * @date    Sep 4 2024
  * @brief   ECE 362 Lab 1 
  ******************************************************************************
*/


/**
******************************************************************************/

// Fill out your username, otherwise your completion code will have the 
// wrong username!
const char* username = "bpar09";

/******************************************************************************
*/ 

#include "stm32f0xx.h"
#include <stdint.h>

void initb();
void initc();
void setn(int32_t pin_num, int32_t val);
int32_t readpin(int32_t pin_num);
void buttons(void);
void keypad(void);
void autotest(void);
extern void internal_clock(void);
extern void nano_wait(unsigned int n);

int main(void) {
    internal_clock(); // do not comment!
    // Comment until most things have been implemented
    autotest();
    initb();
    initc();

    // uncomment one of the loops, below, when ready
    while(1) {
    buttons();
    }

    while(1) {
    keypad();
    1;
  }

    //for(;;);
  
    return 0; //end of main function
}
// Initialize Port B for input and output configuration
//@brief Init GPIO port B
//Pin 0: input
//Pin 4: input
//Pin 8-11: output
void initb(void) {
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN; // Enable clock for Port B

    // Set PB0, PB4 as input (00)
    GPIOB->MODER &= ~(0xF << (0 * 2)); // Clear bits for PB0
    GPIOB->MODER &= ~(0xF << (4 * 2)); // Clear bits for PB4
    
    // Set PB8-PB11 as output (01)
    GPIOB->MODER |= (0x55 << (8 * 2)); // Set bits for PB8, PB9, PB10, PB11
    
    // Clear output data register
    GPIOB->ODR &= ~(0xF << 8);
}

// Initialize Port C for input and output configuration
// @brief Init GPIO port C
//        Pin 0-3: inputs with internal pull down resistors
//        Pin 4-7: outputs

void initc(void) {
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN; // Enable clock for Port C

    // Set PC0-PC3 as input (00) with pull-down resistors
    GPIOC->MODER &= ~(0xFF); // Clear bits for PC0, PC1, PC2, PC3
    GPIOC->PUPDR |= (0xAA);  // Set pull-down resistors for PC0-PC3

    // Set PC4-PC7 as output (01)
    GPIOC->MODER |= (0x55 << (4 * 2)); // Set bits for PC4, PC5, PC6, PC7
}

// Set or clear a pin in GPIOB
//@brief Set GPIO port B pin to some value
//@param pin_num: Pin number in GPIO B
//@param val    : Pin value, if 0 then the
//               pin is set low, else set high
//void setn(int32_t pin_num, int32_t val) 

void setn(int32_t pin_num, int32_t val) {
    if (val) {
        GPIOB->BSRR = (1 << pin_num); // Set pin
    } else {
        GPIOB->BRR = (1 << pin_num);  // Clear pin
    }
}

// Read the value of a pin in GPIOB
//@brief Read GPIO port B pin values
 //@param pin_num   : Pin number in GPIO B to be read
 //@return int32_t  : 1: the pin is high; 0: the pin is low
//int32_t readpin(int32_t pin_num) 
int32_t readpin(int32_t pin_num) {
    return (GPIOB->IDR & (1 << pin_num)) ? 1 : 0; // Return 1 if pin is high, 0 if low
}

// Map buttons to LEDs
//@brief Control LEDs with buttons
//     Use PB0 value for PB8
//     Use PB4 value for PBB
//void buttons(void) 
void buttons(void) {
    int pb0_value = readpin(0);
    int pb4_value = readpin(4);

    setn(8, pb0_value);  // Map PB0 input to PB8 output
    setn(9, pb4_value);  // Map PB4 input to PB9 output
}

// Scan keypad and light up corresponding LEDs
//@brief Control LEDs with keypad
//void keypad(void)
void keypad(void) {
    for (int i = 4; i <= 7; i++) {
        GPIOC->ODR = (1 << i);      // Turn on ith column
        nano_wait(1000000);         // Small delay
        int rows = GPIOC->IDR & 0xF; // Read all rows (PC0-PC3)

        // Check and set corresponding LEDs based on rows
        if (rows & 0x1) setn(8, 1); // Row 1 -> PB8
        if (rows & 0x2) setn(9, 1); // Row 2 -> PB9
        if (rows & 0x4) setn(10, 1);// Row 3 -> PB10
        if (rows & 0x8) setn(11, 1);// Row 4 -> PB11
        
        nano_wait(1000000);         // Small delay
        setn(8, 0);                 // Turn off PB8
        setn(9, 0);                 // Turn off PB9
        setn(10, 0);                // Turn off PB10
        setn(11, 0);                // Turn off PB11
    }
}

// Nano wait function for small delays
void nano_wait(unsigned int n) {
    asm volatile (
        "   mov r0, %[n]      \n"
        "1: sub r0, #1        \n"
        "   bne 1b            \n"
        :
        : [n] "r"(n)
        : "r0", "cc"
    );
}
