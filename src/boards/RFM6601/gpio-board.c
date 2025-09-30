/*!
 * \file      gpio-board.c
 *
 * \brief     Target board GPIO driver implementation
 *
 * \author    Abdullah CAVDAR ( Odak R&D )
 */
/*
#include "stm32l4xx.h"
#include "utilities.h"
#include "sysIrqHandlers.h"
#include "board-config.h"
#include "rtc-board.h"
#include "gpio-board.h"
#if defined( BOARD_IOE_EXT )
#include "gpio-ioe.h"
#endif

*/ 

#include "gpio-board.h"
#include "tremo_gpio.h"
#include "tremo_rcc.h"

static Gpio_t *GpioIrq[16];

void GpioMcuInit(Gpio_t *obj, PinNames pin, PinModes mode, PinConfigs config, PinTypes type, uint32_t value)
{
    if (pin == NC)
        return;

    obj->pin = pin;
    obj->pinIndex = pin % 16;          // 0–15
    obj->portIndex = pin / 16;         // 0=A, 1=B, 2=C...
    obj->Context = NULL;
    obj->IrqHandler = NULL;

    // --- Port selection and clk enable ---
    switch (obj->portIndex) {
        case 0: 
            obj->port = GPIOA; 
            rcc_enable_peripheral_clk(RCC_PERIPHERAL_GPIOA, true); 
            break;

        case 1: 
            obj->port = GPIOB; 
            rcc_enable_peripheral_clk(RCC_PERIPHERAL_GPIOB, true); 
            break;

        case 2: 
            obj->port = GPIOC; 
            rcc_enable_peripheral_clk(RCC_PERIPHERAL_GPIOC, true); 
            break;
            
        default: return; // desteklenmeyen port
    }

    // --- Pull-up/down tipi ---
    gpio_pull_t pull = GPIO_NO_PULL;
    if (type == PIN_PULL_UP)
        pull = GPIO_PULL_UP;
    else if (type == PIN_PULL_DOWN)
        pull = GPIO_PULL_DOWN;

    obj->pull = type;

    // --- Mod seçimi ---
    gpio_mode_t gpioMode;
    switch (mode) {
        case PIN_INPUT:
            gpioMode = GPIO_MODE_INPUT;
            break;
        case PIN_ANALOGIC:
            gpioMode = GPIO_MODE_ANALOG;
            break;
        case PIN_ALTERNATE_FCT:
            gpioMode = GPIO_MODE_FUNCTION;
            break;
        case PIN_OUTPUT:
        default:
            gpioMode = (config == PIN_OPEN_DRAIN)
                       ? GPIO_MODE_OUTPUT_OD
                       : GPIO_MODE_OUTPUT_PP;
            break;
    }

    // --- Init işlemi ---
    uint32_t pinMask = (1 << obj->pinIndex);

    gpio_init(obj->port, pinMask, gpioMode);
    gpio_set_pull_mode(obj->port, pinMask, pull);

    // --- AF ayarı ---
    if (mode == PIN_ALTERNATE_FCT)
        gpio_set_iomux(obj->port, obj->pinIndex, value);

    // --- Başlangıç değeri ---
    if (mode == PIN_OUTPUT)
        gpio_write(obj->port, pinMask, value ? 1 : 0);
}





void GpioMcuSetContext( Gpio_t *obj, void* context )
{
    obj->Context = context;
}

void GpioMcuSetInterrupt( Gpio_t *obj, IrqModes irqMode, IrqPriorities irqPriority, GpioIrqHandler *irqHandler )
{
    if( obj->pin < IOE_0 )
    {
        uint32_t priority = 0;

        IRQn_Type IRQnb = EXTI0_IRQn;
        GPIO_InitTypeDef   GPIO_InitStructure;

        if( irqHandler == NULL )
        {
            return;
        }

        obj->IrqHandler = irqHandler;

        GPIO_InitStructure.Pin =  obj->pinIndex;

        if( irqMode == IRQ_RISING_EDGE )
        {
            GPIO_InitStructure.Mode = GPIO_MODE_IT_RISING;
        }
        else if( irqMode == IRQ_FALLING_EDGE )
        {
            GPIO_InitStructure.Mode = GPIO_MODE_IT_FALLING;
        }
        else
        {
            GPIO_InitStructure.Mode = GPIO_MODE_IT_RISING_FALLING;
        }

        GPIO_InitStructure.Pull = obj->pull;
        GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;

        HAL_GPIO_Init( obj->port, &GPIO_InitStructure );

        switch( irqPriority )
        {
        case IRQ_VERY_LOW_PRIORITY:
        case IRQ_LOW_PRIORITY:
            priority = 3;
            break;
        case IRQ_MEDIUM_PRIORITY:
            priority = 2;
            break;
        case IRQ_HIGH_PRIORITY:
            priority = 1;
            break;
        case IRQ_VERY_HIGH_PRIORITY:
        default:
            priority = 0;
            break;
        }

        switch( obj->pinIndex )
        {
        case GPIO_PIN_0:
            IRQnb = EXTI0_IRQn;
            break;
        case GPIO_PIN_1:
            IRQnb = EXTI1_IRQn;
            break;
        case GPIO_PIN_2:
            IRQnb = EXTI2_IRQn;
            break;
        case GPIO_PIN_3:
            IRQnb = EXTI3_IRQn;
            break;
        case GPIO_PIN_4:
            IRQnb = EXTI4_IRQn;
            break;
        case GPIO_PIN_5:
        case GPIO_PIN_6:
        case GPIO_PIN_7:
        case GPIO_PIN_8:
        case GPIO_PIN_9:
            IRQnb = EXTI9_5_IRQn;
            break;
        case GPIO_PIN_10:
        case GPIO_PIN_11:
        case GPIO_PIN_12:
        case GPIO_PIN_13:
        case GPIO_PIN_14:
        case GPIO_PIN_15:
            IRQnb = EXTI15_10_IRQn;
            break;
        default:
            break;
        }

        GpioIrq[( obj->pin ) & 0x0F] = obj;

        HAL_NVIC_SetPriority( IRQnb , priority, 0 );
        HAL_NVIC_EnableIRQ( IRQnb );
    }
    else
    {
#if defined( BOARD_IOE_EXT )
        // IOExt Pin
        GpioIoeSetInterrupt( obj, irqMode, irqPriority, irqHandler );
#endif
    }
}

void GpioMcuRemoveInterrupt( Gpio_t *obj )
{
    if( obj->pin < IOE_0 )
    {
        // Clear callback before changing pin mode
        GpioIrq[( obj->pin ) & 0x0F] = NULL;

        GPIO_InitTypeDef   GPIO_InitStructure;

        GPIO_InitStructure.Pin =  obj->pinIndex ;
        GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;
        HAL_GPIO_Init( obj->port, &GPIO_InitStructure );
    }
    else
    {
#if defined( BOARD_IOE_EXT )
        // IOExt Pin
        GpioIoeRemoveInterrupt( obj );
#endif
    }
}

void GpioMcuWrite( Gpio_t *obj, uint32_t value )
{
    if( obj->pin < IOE_0 )
    {
        if( obj == NULL )
        {
            assert_param( LMN_STATUS_ERROR );
        }
        // Check if pin is not connected
        if( obj->pin == NC )
        {
            return;
        }
        HAL_GPIO_WritePin( obj->port, obj->pinIndex , ( GPIO_PinState )value );
    }
    else
    {
#if defined( BOARD_IOE_EXT )
        // IOExt Pin
        GpioIoeWrite( obj, value );
#endif
    }
}

void GpioMcuToggle( Gpio_t *obj )
{
    if( obj->pin < IOE_0 )
    {
        if( obj == NULL )
        {
            assert_param( LMN_STATUS_ERROR );
        }

        // Check if pin is not connected
        if( obj->pin == NC )
        {
            return;
        }
        HAL_GPIO_TogglePin( obj->port, obj->pinIndex );
    }
    else
    {
#if defined( BOARD_IOE_EXT )
        // IOExt Pin
        GpioIoeToggle( obj );
#endif
    }
}

uint32_t GpioMcuRead( Gpio_t *obj )
{
    if( obj->pin < IOE_0 )
    {
        if( obj == NULL )
        {
            assert_param( LMN_STATUS_ERROR );
        }
        // Check if pin is not connected
        if( obj->pin == NC )
        {
            return 0;
        }
        return HAL_GPIO_ReadPin( obj->port, obj->pinIndex );
    }
    else
    {
#if defined( BOARD_IOE_EXT )
        // IOExt Pin
        return GpioIoeRead( obj );
#else
        return 0;
#endif
    }
}

void EXTI0_IRQHandler( void )
{
    HAL_GPIO_EXTI_IRQHandler( GPIO_PIN_0 );
}

void EXTI1_IRQHandler( void )
{
    HAL_GPIO_EXTI_IRQHandler( GPIO_PIN_1 );
}

void EXTI2_IRQHandler( void )
{
    HAL_GPIO_EXTI_IRQHandler( GPIO_PIN_2 );
}

void EXTI3_IRQHandler( void )
{
    HAL_GPIO_EXTI_IRQHandler( GPIO_PIN_3 );
}

void EXTI4_IRQHandler( void )
{
    HAL_GPIO_EXTI_IRQHandler( GPIO_PIN_4 );
}

void EXTI9_5_IRQHandler( void )
{
    HAL_GPIO_EXTI_IRQHandler( GPIO_PIN_5 );
    HAL_GPIO_EXTI_IRQHandler( GPIO_PIN_6 );
    HAL_GPIO_EXTI_IRQHandler( GPIO_PIN_7 );
    HAL_GPIO_EXTI_IRQHandler( GPIO_PIN_8 );
    HAL_GPIO_EXTI_IRQHandler( GPIO_PIN_9 );
}

void EXTI15_10_IRQHandler( void )
{
    HAL_GPIO_EXTI_IRQHandler( GPIO_PIN_10 );
    HAL_GPIO_EXTI_IRQHandler( GPIO_PIN_11 );
    HAL_GPIO_EXTI_IRQHandler( GPIO_PIN_12 );
    HAL_GPIO_EXTI_IRQHandler( GPIO_PIN_13 );
    HAL_GPIO_EXTI_IRQHandler( GPIO_PIN_14 );
    HAL_GPIO_EXTI_IRQHandler( GPIO_PIN_15 );
}

void HAL_GPIO_EXTI_Callback( uint16_t gpioPin )
{
    uint8_t callbackIndex = 0;

    if( gpioPin > 0 )
    {
        while( gpioPin != 0x01 )
        {
            gpioPin = gpioPin >> 1;
            callbackIndex++;
        }
    }

    if( ( GpioIrq[callbackIndex] != NULL ) && ( GpioIrq[callbackIndex]->IrqHandler != NULL ) )
    {
        GpioIrq[callbackIndex]->IrqHandler( GpioIrq[callbackIndex]->Context );
    }
}
