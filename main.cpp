
#include "mbed.h"
#include "LPC17xx.h"

// ==================================================
// LPC1768 Timer0 SCL control
//
// Input SCL : P1.26 / CAP0.0
// Output SCL: P1.28 / MAT0.0
//
// Timer clock: SystemCoreClock (PCLK = CCLK)
// ==================================================

static volatile uint32_t delayTicks = 2;
static volatile uint32_t lowTicks   = 1;

static volatile uint32_t nextMatch = 0;
static volatile bool pulseActive = false;

// --------------------------------------------------
// Timing setting
// delayUs : Input SCL rising -> Output SCL falling
// lowUs   : Output SCL Low duration (1.0 - 1.5us)
// --------------------------------------------------

bool setSclTiming(float delayUs, float lowUs)
{
    if (delayUs < 3.0f)
        return false;

    if (lowUs < 1.0f || lowUs > 1.5f)
        return false;

    uint32_t ticksPerUs = SystemCoreClock / 1000000UL;

    __disable_irq();

    delayTicks = (uint32_t)(delayUs * ticksPerUs + 0.5f);
    lowTicks   = (uint32_t)(lowUs * ticksPerUs + 0.5f);

    __enable_irq();

    return true;
}

// --------------------------------------------------
// Timer0 interrupt
// --------------------------------------------------

extern "C" void TIMER0_IRQHandler(void)
{
    uint32_t status = LPC_TIM0->IR;

    // Match 0 interrupt
    if (status & (1 << 0))
    {
        LPC_TIM0->IR = (1 << 0);

        if (!pulseActive)
        {
            // First match:
            // MAT0.0 changes from Hi-Z to LOW

            pulseActive = true;

            // Schedule release
            nextMatch += lowTicks;
            LPC_TIM0->MR0 = nextMatch;
        }
        else
        {
            // Second match:
            // MAT0.0 changes from LOW to Hi-Z

            pulseActive = false;

            // Disable external match action
            LPC_TIM0->EMR &= ~(3 << 4);
        }
    }

    // Capture 0 interrupt (rising edge)
    if (status & (1 << 4))
    {
        uint32_t captured = LPC_TIM0->CR0;

        LPC_TIM0->IR = (1 << 4);

        if (!pulseActive)
        {
            // Enable MAT0.0 toggle action
            LPC_TIM0->EMR =
                (LPC_TIM0->EMR & ~(3 << 4)) | (3 << 4);

            // Schedule first match
            nextMatch = captured + delayTicks;

            LPC_TIM0->MR0 = nextMatch;
        }
    }
}

// --------------------------------------------------
// Timer0 initialization
// --------------------------------------------------

void initSclTimer()
{
    // Enable Timer0 power
    LPC_SC->PCONP |= (1 << 1);

    // Timer0 peripheral clock = CCLK
    LPC_SC->PCLKSEL0 &= ~(3 << 2);
    LPC_SC->PCLKSEL0 |=  (1 << 2);

    // P1.26 = CAP0.0
    LPC_PINCON->PINSEL3 =
        (LPC_PINCON->PINSEL3 & ~(3 << 20)) |
        (3 << 20);

    // P1.28 = MAT0.0
    LPC_PINCON->PINSEL3 =
        (LPC_PINCON->PINSEL3 & ~(3 << 24)) |
        (3 << 24);

    // P1.28 Open Drain
    LPC_PINCON->PINMODE_OD1 |= (1 << 12);

    // Stop and reset Timer0
    LPC_TIM0->TCR = 2;

    LPC_TIM0->PR  = 0;
    LPC_TIM0->TC  = 0;
    LPC_TIM0->PC  = 0;

    LPC_TIM0->MR0 = 0;

    // Match0 interrupt
    LPC_TIM0->MCR = (1 << 0);

    // Capture0 rising edge + interrupt
    LPC_TIM0->CCR =
        (1 << 0) |
        (1 << 2);

    // External Match:
    // EM0 = 1 -> Hi-Z (open drain)
    // EMC0 = 0 -> no action initially
    LPC_TIM0->EMR = (1 << 0);

    // Clear all interrupt flags
    LPC_TIM0->IR = 0x3F;

    pulseActive = false;

    // Enable Timer0 interrupt
    NVIC_ClearPendingIRQ(TIMER0_IRQn);
    NVIC_SetPriority(TIMER0_IRQn, 0);
    NVIC_EnableIRQ(TIMER0_IRQn);

    // Start Timer0
    LPC_TIM0->TCR = 1;
}

// --------------------------------------------------
// Main
// --------------------------------------------------

int main()
{
    // Initial timing
    // Delay = 3us
    // Low   = 1.0us

    setSclTiming(3.0f, 1.0f);

    initSclTimer();

    while (1)
    {
        // Main processing

        // Timing can be changed anytime:
        //
        // setSclTiming(5.0f, 1.2f);
        // setSclTiming(8.0f, 1.5f);
    }
}
