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
#include "RFM6601/cmsis/system_cm4.h"
#include "RFM6601/cmsis/tremo_cm4.h"
#include "mcu/asr6601/driver/peripheral/inc/tremo_gpio.h"
#include "mcu/asr6601/driver/peripheral/inc/tremo_rcc.h"
#include "mcu/asr6601/driver/peripheral/inc/tremo_regs.h"
#include "tremo_gpio.h"
#include "tremo_rcc.h"
#include <stdbool.h>

// asr6601'de A,B,C ve D portları var. Her portta en fazla 16 pin bulunmaktadır.
#define NUM_PORTS 4
#define PINS_PER_PORT 16

static Gpio_t* GpioIrq[NUM_PORTS][PINS_PER_PORT] = {0};

void GpioMcuInit(Gpio_t *obj, PinNames pin, PinModes mode, PinConfigs config,PinTypes type, uint32_t value) 
{
  if (pin < IOE_0) 
  {
    obj->pin = pin;

    if (pin == NC) 
      return;

    obj->pinIndex = obj->pin & 0x0F; //asr6601 de stm32'den farklı

    if ((obj->pin & 0xF0) == 0x00) 
    {
      obj->port = GPIOA;
      rcc_enable_peripheral_clk(RCC_PERIPHERAL_GPIOA, true);
    } 
    else if ((obj->pin & 0xF0) == 0x10) 
    {
      obj->port = GPIOB;
      rcc_enable_peripheral_clk(RCC_PERIPHERAL_GPIOB, true);
    } 
    else if ((obj->pin & 0xF0) == 0x20) 
    {
      obj->port = GPIOC;
      rcc_enable_peripheral_clk(RCC_PERIPHERAL_GPIOC, true);
    } else if ((obj->pin & 0xF0) == 0x30) 
    {
      obj->port = GPIOD;
      rcc_enable_peripheral_clk(RCC_PERIPHERAL_GPIOD, true);
    }

    gpio_mode_t gpio_mode;

    if (mode == PIN_INPUT) 
    {
      if (type == PIN_PULL_UP)
        gpio_mode = GPIO_MODE_INPUT_PULL_UP;
      else if (type == PIN_PULL_DOWN)
        gpio_mode = GPIO_MODE_INPUT_PULL_DOWN;
      else
        gpio_mode = GPIO_MODE_INPUT_FLOATING;
    } 
    else if (mode == PIN_ANALOGIC) 
    {
      gpio_mode = GPIO_MODE_ANALOG;
    } 
    else if (mode == PIN_ALTERNATE_FCT) 
    {
      if (config == PIN_OPEN_DRAIN)
        gpio_mode = GPIO_MODE_OUTPUT_OD_LOW;
      else
        gpio_mode = GPIO_MODE_OUTPUT_PP_LOW;

      // IOMUX set
      gpio_set_iomux((gpio_t*)obj->port, obj->pinIndex, value);
    } 
    else // mode output
    {
      if (config == PIN_OPEN_DRAIN)
        gpio_mode = GPIO_MODE_OUTPUT_OD_LOW;
      else
        gpio_mode = GPIO_MODE_OUTPUT_PP_LOW;
    }

    // Sets initial output value
    if (mode == PIN_OUTPUT) 
    {
      GpioMcuWrite(obj, value);
    }

    gpio_init((gpio_t*)obj->port, obj->pinIndex, gpio_mode);
  } 
  else 
  {
#if defined(BOARD_IOE_EXT)
    // IOExt Pin
    GpioIoeInit(obj, pin, mode, config, type, value);
#endif
  }
}

void GpioMcuSetContext(Gpio_t *obj, void *context) 
{ 
    obj->Context = context; 
}

void GpioMcuSetInterrupt( Gpio_t *obj, IrqModes irqMode, IrqPriorities irqPriority, GpioIrqHandler *irqHandler )
{
    if( obj->pin < IOE_0 )
    {
        uint32_t priority = 0;
        gpio_mode_t gpio_mode;
        gpio_intr_t gpio_intr_type;

        if( irqHandler == NULL )
            return;

        obj->IrqHandler = irqHandler;
        obj->pinIndex = obj->pin & 0x0F;

        if( irqMode == IRQ_RISING_EDGE )
        {
            gpio_intr_type = GPIO_INTR_RISING_EDGE;
        }
        else if( irqMode == IRQ_FALLING_EDGE )
        {
            gpio_intr_type = GPIO_INTR_FALLING_EDGE;
        }
        else
        {
            gpio_intr_type = GPIO_INTR_RISING_FALLING_EDGE;
        }

        if(obj->pull == PIN_PULL_UP)
            gpio_mode = GPIO_MODE_INPUT_PULL_UP;
        else if(obj->pull == PIN_PULL_DOWN)
            gpio_mode = GPIO_MODE_INPUT_PULL_DOWN;
        else
            gpio_mode = GPIO_MODE_INPUT_FLOATING;

        gpio_init((gpio_t*)obj->port, obj->pinIndex, gpio_mode);
        gpio_config_interrupt((gpio_t*)obj->port, obj->pinIndex, gpio_intr_type);

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

        /* NVIC config */
        NVIC_EnableIRQ(GPIO_IRQn);
        NVIC_SetPriority(GPIO_IRQn, priority);

        GpioIrq[obj->pin & 0xF0][obj->pin & 0x0F] = obj;  //asr6601'de bütün interruptlar aynı yere yönlendirilmiş.
    }
    else
    {
#if defined( BOARD_IOE_EXT )
        // IOExt Pin
        GpioIoeSetInterrupt( obj, irqMode, irqPriority, irqHandler );
#endif
    }
}

void GpioMcuRemoveInterrupt(Gpio_t *obj) 
{
    if (obj->pin < IOE_0) 
    {
        // Clear callback before changing pin mode
        GpioIrq[obj->pin & 0xF0][obj->pin & 0x0F] = NULL;

        gpio_mode_t gpio_mode = GPIO_MODE_ANALOG;
        obj->pinIndex = obj->pin & 0x0F;
        gpio_init((gpio_t*)obj->port, obj->pinIndex, gpio_mode);
    } 
    else 
    {
#if defined(BOARD_IOE_EXT)
        // IOExt Pin
        GpioIoeRemoveInterrupt(obj);
#endif
    }
}

void GpioMcuWrite(Gpio_t *obj, uint32_t value) 
{
  if (obj->pin < IOE_0) 
  {
    if (obj == NULL) 
    {
        assert_param(LMN_STATUS_ERROR);
    }
    // Check if pin is not connected
    if (obj->pin == NC)
        return;
    
    obj->pinIndex = obj->pin & 0x0F;
    gpio_write((gpio_t*)obj->port, obj->pinIndex, (gpio_level_t)value);
  } 
  else 
  {
#if defined(BOARD_IOE_EXT)
    // IOExt Pin
    GpioIoeWrite(obj, value);
#endif
  }
}

void GpioMcuToggle(Gpio_t *obj) 
{
  if (obj->pin < IOE_0) 
  {
    if (obj == NULL) 
    {
      assert_param(LMN_STATUS_ERROR);
    }

    // Check if pin is not connected
    if (obj->pin == NC) 
      return;
    
    obj->pinIndex = obj->pin & 0x0F;
    gpio_toggle( (gpio_t*)obj->port, obj->pinIndex);
  } 
  else 
  {
#if defined(BOARD_IOE_EXT)
    // IOExt Pin
    GpioIoeToggle(obj);
#endif
  }
}

uint32_t GpioMcuRead(Gpio_t *obj) {
  if (obj->pin < IOE_0)
  {
    if (obj == NULL) 
    {
      assert_param(LMN_STATUS_ERROR);
    }
    // Check if pin is not connected
    if (obj->pin == NC) 
      return 0;

    obj->pinIndex = obj->pin & 0x0F;
    return gpio_read((gpio_t*)obj->port, obj->pinIndex);
  } 
  else 
  {
#if defined(BOARD_IOE_EXT)
    // IOExt Pin
    return GpioIoeRead(obj);
#else
    return 0;
#endif
  }
}

void TREMO_GPIO_Callback(void)
{
    for (uint8_t portIndex = 0; portIndex < NUM_PORTS; portIndex++)  // GPIOA–GPIOD
    {
        gpio_t *gpiox = NULL;

        switch (portIndex)
        {
            case 0: gpiox = GPIOA; break;
            case 1: gpiox = GPIOB; break;
            case 2: gpiox = GPIOC; break;
            case 3: gpiox = GPIOD; break;
        }

        for (uint8_t pinIndex = 0; pinIndex < PINS_PER_PORT; pinIndex++)
        {
            // Check if the interrupt flag is set for this pin
            if (gpio_get_interrupt_status(gpiox, pinIndex) == SET)
            {
                gpio_clear_interrupt(gpiox, pinIndex);
                // If a callback is registered for this pin, call it
                if (GpioIrq[portIndex][pinIndex] != NULL &&
                    GpioIrq[portIndex][pinIndex]->IrqHandler != NULL)
                {
                    GpioIrq[portIndex][pinIndex]->IrqHandler(GpioIrq[portIndex][pinIndex]->Context);
                }
            }
        }
    }
}

void GPIO_IRQHandler(void)
{
    TREMO_GPIO_Callback();
}
